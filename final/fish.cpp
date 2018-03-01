
Vec3f separate() {
  int count = 0;
  Vec3f steer;

  for (auto other : *particles) {
    // this difference is a vector from b to a, put this force on a, so
    // push away.
    Vec3f difference = (pose.pos() - other.pose.pos());
    float d = difference.mag();
    // if agents is getting closer, push away
    if (d > 0 && d < 6 * sphereRadius) {
      steer += difference.normalize() / d;
      count++;
    }
  }
  if (count > 0) {
    steer = steer / count;
  }
  if (steer.mag() > 0) {
    steer.normalize(maxSpeed);
    steer -= velocity;
  }
  return steer;
}

// need different draw for each sub class
// new alive flag
