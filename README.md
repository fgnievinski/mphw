# MPHW – Multipath Hardware
**An open-source low-cost sensor for SNR-based GPS/GNSS Reflectometry (GNSS-R)**

*Tutorial*: [PDF](https://github.com/fgnievinski/mphw/blob/master/docs/Tutorial%20MPHW%20GNSS-R.pdf) [HTML](https://docs.google.com/document/d/e/2PACX-1vQWZE0kOz02pycYrD1TnTDzAgdpxIq0RMhyQfiYGw8P_tUOE7rEuvWMlt8Ty0SoXUEcO8JzxejXl3Y9/pub)

[List of materials](https://www.adafruit.com/wishlists/469752)
(see note below about GPS replacement)

Please give credit citing Fagundes et al. (2021):

**M. A. R. Fagundes, I. Mendonça-Tinti, A. L. Iescheck, D. M. Akos, F. Geremia-Nievinski (2021), "An open-source low-cost sensor for SNR-based GNSS relectometry: design and long-term validation towards sea-level altimetry" *GPS Solutions*, https://doi.org/10.1007/s10291-021-01087-1**

Previous works:

Preprint: Fagundes, M.; Mendonça-Tinti, I.; Iescheck, A.; Akos, D.; Geremia-Nievinski, F. (2020) "An open-source low-cost sensor for SNR-based GNSS reflectometry: Design and long-term validation towards sea level altimetry". *Preprint*, available at (https://www.researchgate.net/publication/341946011)

Poster: Fagundes, M.A.R.; Geremia-Nievinski, F. (2019) "Open-source hardware options for SNR based GPS/GNSS reflectometry – Proof of concept and initial validation", *IEEE GNSS+R 2019*, Specialist Meeting on Reflectometry using GNSS and other Signals of Opportunity, 20-22 May 2019, Benevento, Italy. 

![poster](https://raw.githubusercontent.com/fgnievinski/mphw/master/docs/poster_pre_revisao7b.jpg)

Note about the GPS: in 2021 Adafruit updated their GPS FeatherWing product -- without changing the product identifier ([PID 3133](https://www.adafruit.com/product/3133)): the GPS receiver module, originally GlobalTop's PA6H (based on MediaTek MTK3339), was replaced by [CDtop's PA1616D](https://www.cdtop-tech.com/products/pa1616d) (based on MediaTek's MTK3333). The most similar product currently available from Adafruit is their GPS breakout PA1616S ([PID 746](https://www.adafruit.com/product/746)), using [CDtop's PA1616S](https://www.cdtop-tech.com/products/pa1616s) (also based on MediaTek MTK3339). We have updated the shopping list to include both Adafruit products (FeatherWing 3133 and Breakout 746). We recommend this compromise solution as the tutorial describes how to use the GPS FeatherWing, while this particular GPS Breakout (and not any other similar ones) is strictly equivalent to the GPS we have tested. In the near future, we plan to update the tutorial to use the Breakout instead of the Featherwing. Further into the future, we plan to check if MTK3333 could replace MTK3339, which may require source code modifications. In principle, MTK3333 is superior to MTK3339, as it supports GLONASS in addition to GPS; however, we have not tested MKT3333 and we have custom firmware for MKT3339 to output NMEA with C/No or SNR as non-integer decimal values.
