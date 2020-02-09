#ifndef ROBOTGEOMETRY_H_
#define ROBOTGEOMETRY_H_

#define THICK_ARM_LEN    178.0

class RobotGeometry {
public:
  RobotGeometry();
  void set(float axmm, float aymm, float azmm);
  void from(float arot, float alow, float ahigh);
  float getXmm() const;
  float getYmm() const;
  float getZmm() const;
  float getRotRad() const;
  float getLowRad() const;
  float getHighRad() const;
private:
  void calculateGrad();
  void calculateAxis();
  float xmm;
  float ymm;
  float zmm;
  float rot;
  float low;
  float high;
};

#endif

