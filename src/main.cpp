// ═══════════════════════════════════════════════════════════════════════════
// Модуль 2.6 · домашнє завдання · брязкіт кнопки: 4 способи з домашки 2.4 на ESP-IDF
// Плата: YD-ESP32-S3 (ESP32-S3-WROOM-1-N16R8) · PlatformIO · framework = espidf
//
// Логіка та сама, що в 2.4. Спосіб обирається клавішею в моніторі порту:
//   1 — без захисту: кожен FALLING з ISR рахується
//   2 — time-based: ISR лише прапорець, цикл приймає, якщо минуло ≥ 50 мс
//   3 — state-based: ISR лише прапорець, цикл приймає, якщо кнопка ДОСІ натиснута
//   4 — polling без переривань: читати пін кожні 5 мс, машина станів
//   u — увімкнути/вимкнути внутрішню підтяжку (для RC-фільтра з зовнішньою 10 кОм)
//   r — обнулити лічильники · ? — підказка
//
// Залізо без змін: кнопка з RC на GPIO4, LED на GPIO18.
// ═══════════════════════════════════════════════════════════════════════════
#include <fcntl.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
    // ── 1. Конфігурація ──────────────────────────────────────────────────────────
    struct Config {
        static constexpr gpio_num_t BTN_PIN = GPIO_NUM_4; // кнопка між піном і GND
        static constexpr gpio_num_t LED_PIN = GPIO_NUM_18; // перемикається на кожне прийняте натискання

        static constexpr uint32_t DEBOUNCE_MS = 50; // спосіб 2: вікно тиші
        static constexpr uint32_t POLL_MS = 5; // спосіб 4: як часто читати пін
        static constexpr uint8_t STABLE_READS = 4; // спосіб 4: 4 × 5 мс = 20 мс однакового рівня
    };

    enum class Method : uint8_t { Raw = 1, Time = 2, State = 3, Poll = 4 };

    const char *toText(const Method m) {
        switch (m) {
            case Method::Raw: return "1 без захисту";
            case Method::Time: return "2 time-based";
            case Method::State: return "3 state-based";
            case Method::Poll: return "4 polling";
        }

        return "?";
    }

    // ── 2. Те, що пише ISR ───────────────────────────────────────────────────────
    volatile uint32_t isrEdges = 0; // скільки FALLING побачило залізо загалом
    volatile bool edgePending = false; // «був фронт, цикл ще не забрав»
    volatile int64_t firstEdgeUs = 0; // 64 біти: процесор читає їх за два кроки, тому лише в критичній секції

    // Замість noInterrupts(): спінлок. Двоядерний чип, вимкнути переривання одного ядра замало.
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // ISR: лише лічильник, прапорець і час. Жодного printf, затримок, логіки.
    void IRAM_ATTR onFalling(void *) {
        portENTER_CRITICAL_ISR(&mux);

        if (!edgePending) // час фіксуємо лише для першого фронту пачки
            firstEdgeUs = esp_timer_get_time();

        edgePending = true;

        isrEdges = isrEdges + 1; // C++20 забороняє ++ для volatile: це насправді два кроки, читання і запис

        portEXIT_CRITICAL_ISR(&mux);
    }

    // Забрати подію неподільно.
    struct Edge {
        bool pending;
        int64_t firstUs;
    };

    Edge takeEdge() {
        Edge e{
            .pending = false,
            .firstUs = 0
        };

        portENTER_CRITICAL(&mux);

        if (edgePending) {
            e = {
                .pending = true,
                .firstUs = firstEdgeUs
            };
            edgePending = false;
        }

        portEXIT_CRITICAL(&mux);

        return e;
    }

    uint32_t readIsrEdges() {
        portENTER_CRITICAL(&mux);

        const uint32_t n = isrEdges;

        portEXIT_CRITICAL(&mux);

        return n;
    }

    // ── 3. Стан програми ─────────────────────────────────────────────────────────
    auto method = Method::Raw;
    bool internalPullup = true;
    uint32_t accepted = 0; // скільки натискань прийняв поточний спосіб
    bool ledOn = false;
    bool isrAttached = false;

    bool buttonDown() {
        return gpio_get_level(Config::BTN_PIN) == 0;
    } // підтяжка вгору: натиснута = 0

    uint32_t nowMs() {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000);
    }

    // Спільне «прийняли натискання»: лічильник, LED, рядок у лог.
    void accept(const int64_t latencyUs) {
        ++accepted;
        ledOn = !ledOn;
        gpio_set_level(Config::LED_PIN, ledOn ? 1 : 0);

        printf(
            "[%s] натискань: %lu · фронтів ISR усього: %lu · затримка %lu мкс\n", toText(method),
            static_cast<unsigned long>(accepted), static_cast<unsigned long>(readIsrEdges()),
            static_cast<unsigned long>(latencyUs)
        );
    }

    void resetCounters() {
        portENTER_CRITICAL(&mux);

        isrEdges = 0;
        edgePending = false;

        portEXIT_CRITICAL(&mux);

        accepted = 0;
    }

    void applyInput() {
        if (internalPullup)
            gpio_pullup_en(Config::BTN_PIN);
        else
            gpio_pullup_dis(Config::BTN_PIN);
    }

    void setMethod(const Method m) {
        method = m;
        // Спосіб 4 працює без переривань — тоді від'єднуємо ISR повністю.
        if (m == Method::Poll && isrAttached) {
            gpio_isr_handler_remove(Config::BTN_PIN);
            isrAttached = false;
        } else if (m != Method::Poll && !isrAttached) {
            gpio_isr_handler_add(Config::BTN_PIN, onFalling, nullptr);
            isrAttached = true;
        }

        resetCounters();

        printf(
            "── спосіб %s · підтяжка %s · лічильники обнулено ──\n", toText(method),
            internalPullup ? "внутрішня" : "зовнішня"
        );
    }

    void printHelp() {
        printf("1 без захисту · 2 time · 3 state · 4 polling · u підтяжка · r обнулити · ? підказка\n");
    }

    // ── 4. Способи 2, 3, 4 (спосіб 1 — це сам лічильник ISR) ────────────────────
    void runTime(const uint32_t now) {
        static uint32_t lastAccepted = 0;

        const Edge e = takeEdge();
        if (e.pending && now - lastAccepted >= Config::DEBOUNCE_MS) {
            lastAccepted = now;
            accept(esp_timer_get_time() - e.firstUs);
        }
    }

    void runState() {
        static bool counted = false; // це натискання вже зараховане?

        const bool down = buttonDown();
        if (!down)
            counted = false; // відпустили → чекаємо нового натискання

        const Edge e = takeEdge();
        if (e.pending && down && !counted) {
            counted = true;
            accept(esp_timer_get_time() - e.firstUs);
        }
    }

    // Машина станів: рівень мусить протриматись STABLE_READS читань поспіль.
    enum class PollState : uint8_t { Released, MaybePressed, Pressed, MaybeReleased };

    void runPoll(const uint32_t now) {
        static uint32_t lastRead = 0;
        static auto st = PollState::Released;
        static uint8_t same = 0;
        static int64_t firstLowUs = 0;

        if (now - lastRead < Config::POLL_MS)
            return;

        lastRead = now;

        const bool down = buttonDown();
        switch (st) {
            case PollState::Released:
                if (down) {
                    st = PollState::MaybePressed;
                    same = 1;
                    firstLowUs = esp_timer_get_time();
                }

                break;
            case PollState::MaybePressed:
                if (!down) {
                    st = PollState::Released; // це був відскок
                } else if (++same >= Config::STABLE_READS) {
                    st = PollState::Pressed;
                    accept(esp_timer_get_time() - firstLowUs);
                }

                break;
            case PollState::Pressed:
                if (!down) {
                    st = PollState::MaybeReleased;
                    same = 1;
                }

                break;
            case PollState::MaybeReleased:
                if (down) {
                    st = PollState::Pressed; // відскок відпускання
                } else if (++same >= Config::STABLE_READS) {
                    st = PollState::Released;
                }

                break;
        }
    }

    void handleKeys() {
        char key = 0;
        if (read(STDIN_FILENO, &key, 1) != 1) // немає символа — read одразу повертається
            return;

        switch (key) {
            case '1':
                setMethod(Method::Raw);

                break;
            case '2':
                setMethod(Method::Time);

                break;
            case '3':
                setMethod(Method::State);

                break;
            case '4':
                setMethod(Method::Poll);

                break;
            case 'u':
                internalPullup = !internalPullup;

                applyInput();
                setMethod(method);

                break;
            case 'r':
                resetCounters();

                printf("── лічильники обнулено ──\n");

                break;
            case '?':
                printHelp();

                break;
            default:
                break; // Enter, пробіли тощо — ігноруємо
        }
    }

    void setupPins() {
        const gpio_config_t led{
            .pin_bit_mask = 1ULL << Config::LED_PIN,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&led));
        gpio_set_level(Config::LED_PIN, 0);

        const gpio_config_t btn{
            .pin_bit_mask = 1ULL << Config::BTN_PIN,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE, // FALLING
        };
        ESP_ERROR_CHECK(gpio_config(&btn));

        ESP_ERROR_CHECK(gpio_install_isr_service(0)); // спільна служба переривань GPIO
    }
} // namespace

// extern "C": IDF шукає app_main за іменем C, а C++ це ім'я інакше кодує
extern "C" void app_main() {
    // Без драйвера консоль лише друкує: прийняті байти read() не бачить
    usb_serial_jtag_driver_config_t usbConfig = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usbConfig));
    usb_serial_jtag_vfs_use_driver();
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // щоб read() не чекав на клавішу


    setupPins();

    setMethod(Method::Raw); // переривання підключається тут, останнім

    printHelp();

    // app_main уже є задачею FreeRTOS: цикл живе тут, нових задач не створюємо
    while (true) {
        const uint32_t now = nowMs();

        handleKeys();

        switch (method) {
            case Method::Raw: {
                // Без захисту: кожен фронт = натискання. Друкуємо, коли лічильник змінився.
                static uint32_t shown = 0;

                takeEdge(); // прапорець тут не потрібен — лише скидаємо

                const uint32_t edges = readIsrEdges();

                if (edges != shown) {
                    shown = edges;
                    accepted = edges;
                    printf("[%s] натискань (= фронтів): %lu\n", toText(method), static_cast<unsigned long>(edges));
                }

                break;
            }
            case Method::Time:
                runTime(now);

                break;
            case Method::State:
                runState();

                break;
            case Method::Poll:
                runPoll(now);

                break;
        }

        vTaskDelay(1); // віддати процесор: без паузи спрацює сторожовий таймер задачі
    }
}
