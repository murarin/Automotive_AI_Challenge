#ifndef RECTCLASSSCORE_H_
#define RECTCLASSSCORE_H_

#include <sstream>
#include <string>

template<typename _Tp>
class RectClassScore
{
public:
  _Tp x, y, w, h;
  _Tp score;
  unsigned int class_type;
  bool enabled;

  inline std::string toString()
  {
    std::ostringstream out;
    out << "P(" << GetClassString() << ") at " << "(x:" << x << ", y:" << y << ", w:" << w << ", h:" << h << ") ="
        << score;
    return out.str();
  }

  inline std::string GetClassString()
  {
    switch (class_type)
    {
      case 0:
        return "car";
    }
  }
};


#endif /* RECTCLASSSCORE_H_ */
