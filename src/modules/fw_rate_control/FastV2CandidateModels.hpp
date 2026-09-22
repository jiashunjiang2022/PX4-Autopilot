#pragma once
// AUTO-GENERATED FROM FAST_V2C_REGISTERB_6F_20260922. Shadow-only.
namespace fast_v2c {
namespace abs4 {
constexpr float mean[4]={7.338845730e-02f,7.326698303e-02f,5.798853561e-02f,6.801700592e-02f};
constexpr float scale[4]={1.108038649e-01f,1.107943356e-01f,4.645145833e-01f,6.251754165e-01f};
constexpr float coef[4]={4.377522692e-02f,-4.531421140e-02f,4.249868914e-03f,1.844390645e-03f};
constexpr float bias=3.889493819e-04f;
inline float predict(const float x[4]) { float y=bias; for(int i=0;i<4;++i) { const float z=(x[i]-mean[i])/scale[i]; y=y+coef[i]*z; } return y; }
}
namespace delta4 {
constexpr float mean[4]={1.214680451e-04f,2.383614046e-04f,5.798853561e-02f,6.801700592e-02f};
constexpr float scale[4]={4.388537258e-03f,7.146112621e-03f,4.645145833e-01f,6.251754165e-01f};
constexpr float coef[4]={1.997328829e-03f,1.470724121e-03f,2.879725071e-03f,1.864307676e-03f};
constexpr float bias=3.889493819e-04f;
inline float predict(const float x[4]) { float y=bias; for(int i=0;i<4;++i) { const float z=(x[i]-mean[i])/scale[i]; y=y+coef[i]*z; } return y; }
}
}
