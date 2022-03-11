# use hsv color to represent numbers
# translated from:
# https://stackoverflow.com/questions/46928277/trying-to-convert-integer-range-to-rgb-color
def transitionOfHueRange(percentage, startHue, endHue):
    hue = ((percentage * (endHue - startHue)) + startHue) / 360
    saturation = 1.0
    lightness = 0.5
    return hslColorToRgb(hue, saturation, lightness)

def hslColorToRgb(hue, saturation, lightness):
    if saturation == 0.0:
        # The color is achromatic (has no color)
        # Thus use its lightness for a grey-scale color
        grey = percToColor(lightness)
        return (grey, grey, grey)

    if lightness < 0.5:
        q = lightness * (1 + saturation)
    else:
        q = lightness + saturation - lightness * saturation
    p = 2 * lightness - q

    oneThird = 1.0 / 3
    red = percToColor(hueToRgb(p, q, hue + oneThird))
    green = percToColor(hueToRgb(p, q, hue))
    blue = percToColor(hueToRgb(p, q, hue - oneThird))

    return (red, green, blue)

def hueToRgb( p,  q,  t):
    if t < 0:
        t += 1
    if t > 1:
        t -= 1
    if t < 1.0 / 6:
        return p + (q - p) * 6 * t
    
    if t < 1.0 / 2:
        return q
    
    if t < 2.0 / 3:
        return p + (q - p) * (2.0 / 3 - t) * 6
    
    return p

def percToColor(percentage):
    return percentage
    #return round(percentage * 255)