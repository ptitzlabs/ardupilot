// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_location_to_vector(const Location& loc)
{
    Vector3f tmp((loc.lat-home.lat) * LATLON_TO_CM, (loc.lng-home.lng) * LATLON_TO_CM * scaleLongDown, loc.alt);
    return tmp;
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}
