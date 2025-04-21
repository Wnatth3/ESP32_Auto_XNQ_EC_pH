#ifndef MenuList_h
#define MenuList_h

#include "Arduino.h"
#include "Header.h"

//-----Custom floatField----------------
#define DECIMALSFLIED_DEFAULT 1  // number of decimals
template <typename T>
class decimalslField
    : public menuField<T> {  // https://github.com/neu-rah/ArduinoMenu/blob/master/examples/customField/customField/customField.ino
   private:
    idx_t decimals;

   public:
    decimalslField(constMEM menuFieldShadow<T> &shadow) : menuField<T>(shadow) { decimals = DECIMALSFLIED_DEFAULT; }
    decimalslField(
        T         &value,
        constText *text,
        constText *units,
        T          low,
        T          high,
        T          step,
        T          tune,
        action     a = doNothing,
        eventMask  e = noEvent,
        styles     s = noStyle) : decimalslField(*new menuFieldShadow<T>(value, text, units, low, high, step, tune, a, e, s)) {}

    Used printTo(navRoot &root, bool sel, menuOut &out, idx_t idx, idx_t len, idx_t panelNr = 0) override {  // https://github.com/neu-rah/ArduinoMenu/issues/94#issuecomment-290936646
        // menuFieldShadow<T>& s=*(menuFieldShadow<T>*)shadow;
        menuField<T>::reflex = menuField<T>::target();
        idx_t l              = prompt::printTo(root, sel, out, idx, len);
        // bool ed = this == root.navFocus;
        // bool sel=nav.sel==i;
        if (l < len) {
            out.print((root.navFocus == this && sel) ? (menuField<T>::tunning ? '>' : ':') : ' ');
            l++;
            if (l < len) {
                l += out.print(menuField<T>::reflex, decimals);  // NOTE: this can exceed the limits!
                if (l < len) {
                    l += print_P(out, fieldBase::units(), len);
                }
            }
        }
        return l;
    }

    void  setDecimals(idx_t d) { decimals = d; }
    idx_t getDecimals(void) { return (decimals); }
};

//----------------- Menu ----------------------//
using namespace Menu;

#define MAX_DEPTH 3

// Define the display------------------
#define fontName  u8g2_font_6x10_mf
#define fontX     6
#define fontY     12
#define offsetX   0
#define offsetY   1
#define U8_Width  128
#define U8_Height 64

// U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);  // ,22, 21); // page buffer, size = 128 bytes
#ifdef simulation
U8G2_ST7567_ENH_DG128064I_1_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
#else
U8G2_ST7567_ENH_DG128064I_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

// define menu colors -----------------
const colorDef<uint8_t> colors[6] MEMMODE = {
    {{0, 0}, {0, 1, 1}}, // bgColor
    {{1, 1}, {1, 0, 0}}, // fgColor
    {{1, 1}, {1, 0, 0}}, // valColor
    {{1, 1}, {1, 0, 0}}, // unitColor
    {{0, 1}, {0, 0, 1}}, // cursorColor
    {{1, 1}, {1, 0, 0}}, // titleColor
};

// Click Encoder
#define encA     35  // 16  // CLK pin of click encoder
#define encB     34  // 17  // DT pin of click encoder
#define encBtn   39  // 4   // SW pin of click encoder
#define encSteps 2

ClickEncoder       clickEncoder = ClickEncoder(encA, encB, encBtn, encSteps);
ClickEncoderStream encStream(clickEncoder, 1);

// void IRAM_ATTR onTimer();

// void savUChar(const char *kName, uint8_t val, const char *pubTopic);
// void savUShort(const char *kName, uint16_t val, const char *pubTopic);
// void savFloat(const char *kName, float val, const char *pubTopic);

// void savState(); 
// void savWaterLvMax(); 
// void savWaterLvMin(); 
// void savEcMax();
// void savEcMin(); 
// void savFertAPumpPct(); 
// void savFertBPumpPct(); 
// void savFertOn(); 
// void savFertOff();
// void savPhMax(); 
// void savPhMin(); 
// void savPhOn(); 
// void savPhOff();


// void espRestart();



#endif