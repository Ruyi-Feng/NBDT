# Dataset Metadata

This document organizes the **Meta Data** and **Intermediate Variables** for 5 trajectory datasets (CitySim, highD, inD, rounD, NGSIM). Format reference: [TrajectoryDataFormat Wiki](https://github.com/ZhilingResearch/Ozone/wiki/TrajectoryDataFormat).

---

## 1. CitySim

**Application Page**: https://github.com/UCF-SST-Lab/UCF-SST-CitySim1-Dataset

**Data Source**: University of Central Florida (UCF), USA / Partner universities (Southwest Jiaotong University, Southeast University, Hong Kong Polytechnic University)

### 1.1 Meta Data

| Field | Value |
|-------|-------|
| datasetName | CitySim |
| siteName | See site list below |
| recordingDate | (not provided by dataset) |
| weekDay | (not provided by dataset) |
| localWeather | (pending local historical weather lookup) |
| recordingTime | (not provided by dataset) |
| recordingFrameRate | 30 FPS |
| totalFrames | See recording details below |
| duration | ~1140 minutes of drone video in total |
| map | Per-scene background maps in `main/CitySim/` |
| laneRange | laneId field available in data |

**Site List** (13 scenes):

| Scene | Road Type | Country |
|-------|-----------|---------|
| IntersectionA | Intersection (University@Alafaya, Signalized) | USA |
| IntersectionB | Intersection (McCulloch@Seminole, Non-signalized) | USA |
| IntersectionC | Intersection (University@McCulloch, Signalized) | USA |
| IntersectionD | Intersection (GarageC, Consecutive signalized) | USA |
| IntersectionE | Intersection (Permissive left turn phasing) | USA |
| IntersectionF | Intersection (Non-signalized) | USA |
| RoundaboutA | Roundabout (Single lane) | USA |
| RoundaboutB | Roundabout (Two lane) | USA |
| ExpresswayA | Expressway (Weaving segment) | China |
| ExpresswayB | Expressway (Weaving segment) | China |
| FreewayB | Freeway (Basic segment) | China |
| FreewayC | Freeway (Merge/diverge) | China |
| FreewayD | Freeway (Merge/diverge) | China |

The coordinate relationship between the provided trajectory position and the base map is shown in the figure.

![CitySim Coordinate System](images/coord_CitySim.png)

### 1.2 Intermediate Variables

| Variable | Value |
|----------|-------|
| pix2meter | ExpresswayA: 17.912853 pixel = 1 meter; other scenes: — |
| imgLon\*1, imgLat\*1 | (no GPS coordinates provided) |
| imgLon\*2, imgLat\*2 | — |
| imgLon\*3, imgLat\*3 | — |
| imgLon\*4, imgLat\*4 | — |

> **Note**: pix2meter was derived from the ratio of pixel coordinates to feet coordinates in the data: `carCenterX (pixel) / (carCenterXft * 0.3048)`, with zero variance (std=0).

---

## 2. highD

**Application Page**: https://levelxdata.com/highd-dataset/

**Data Source**: RWTH Aachen University (ika), German Autobahn

### 2.1 Meta Data

| Field | Value |
|-------|-------|
| datasetName | highD |
| siteName | weisweiler, garzweiler, grevenbroich, bergheim-sud, serways-raststatte, koln-west (Highway, Germany) |
| recordingDate | Sep 2017 – Jul 2018 (month-level precision, see recording details) |
| weekDay | Tue, Thu, Fri, Mon, Wed |
| localWeather | Sunny and windless |
| recordingTime | See recording details table |
| recordingFrameRate | 25 FPS |
| totalFrames | Varies per recording (= duration × 25) |
| duration | 389 – 1251 seconds |
| map | XX_highway.png (one per recording) |
| laneRange | upperLaneMarkings / lowerLaneMarkings (see recordingMeta) |

**Recording Details** (60 recordings across 11 days):

| month | weekDay | startTime | duration (min) |
|-------|---------|-----------|----------------|
| 9.2017 | Tue | 08:38 | 49 |
| 9.2017 | Thu | 11:16 | 57 |
| 9.2017 | Thu | 16:18 | 62 |
| 9.2017 | Fri | 08:21 | 56 |
| 9.2017 | Fri | 08:49 | 143 |
| 10.2017 | Mon | 08:55 | 182 |
| 10.2017 | Mon | 09:04 | 131 |
| 10.2017 | Wed | 11:26 | 76 |
| 11.2017 | Wed | 08:47 | 144 |
| 1.2018 | Thu | 09:16 | 69 |
| 7.2018 | Wed | 09:15 | 30 |

The coordinate relationship between the provided trajectory position and the base map is shown in the figure.

![highD Coordinate System](images/coord_highD.png)

### 2.2 Intermediate Variables

| Variable | Value |
|----------|-------|
| pix2meter | — |
| imgLon\*1, imgLat\*1 | (highD does not provide GPS coordinates) |
| imgLon\*2, imgLat\*2 | — |
| imgLon\*3, imgLat\*3 | — |
| imgLon\*4, imgLat\*4 | — |

---

## 3. inD

**Application Page**: https://levelxdata.com/ind-dataset/

**Data Source**: RWTH Aachen University (ika), German urban intersections

### 3.1 Meta Data

| Field | Value |
|-------|-------|
| datasetName | inD |
| siteName | Bendplatz, Frankenburg, Heckstrasse, Neukollner Strasse (Intersection, Germany) |
| recordingDate | (dataset only provides weekday, no specific date) |
| weekDay | monday, tuesday, wednesday, thursday |
| localWeather | (pending local historical weather lookup in Aachen, Germany) |
| recordingTime | startTime field (hour of day, see recording details) |
| recordingFrameRate | 25 FPS |
| totalFrames | Varies per recording (= duration × 25) |
| duration | 648 – 1328 seconds |
| map | XX_background.png (one per recording) |
| laneRange | Lanelet map files available (OSM format) |

**Recording Details** (33 recordings across 9 days):

| weekday | startTime | duration (min) | localWeather |
|---------|-----------|----------------|-------------|
| wednesday | 16:00 | 16 | — |
| tuesday | 15:00 | 32 | — |
| monday | 12:00 | 62 | — |
| tuesday | — | 63 | — |
| monday | 16:00 | 67 | — |
| tuesday | 15:00 | 56 | — |
| tuesday | 16:00 | 135 | — |
| wednesday | 16:00 | 107 | — |
| thursday | 13:00 | 51 | — |

The coordinate relationship between the provided trajectory position and the base map is shown in the figure.

![inD Coordinate System](images/coord_inD.png)

### 3.2 Intermediate Variables

| Variable | locationId 1 (Bendplatz) | locationId 2 (Frankenburg) | locationId 3 (Heckstrasse) | locationId 4 (Neukollner Str.) |
|----------|--------------------------|---------------------------|---------------------------|-------------------------------|
| pix2meter | 122.76 | 122.76 | 122.76 | 78.74 |
| xUtmOrigin | 293487.1224 | 295620.9575 | 300127.0853 | 297631.3187 |
| yUtmOrigin | 5629712 | 5628102 | 5629091 | 5629917 |
| latLocation | 50.78207 | 50.76836 | 50.77887 | 50.78505 |
| lonLocation | 6.07116 | 6.10227 | 6.16553 | 6.13070 |
| imgLon\*1–4, imgLat\*1–4 | (to be derived from UTM origin + image size × pix2meter) | — | — | — |


---

## 4. rounD

**Application Page**: https://levelxdata.com/round-dataset/

**Data Source**: RWTH Aachen University (ika), German roundabouts

### 4.1 Meta Data

| Field | Value |
|-------|-------|
| datasetName | rounD |
| siteName | Thiergarten, KackertstraBe, Neuweiler (Roundabout, Germany) |
| recordingDate | (dataset only provides weekday, no specific date) |
| weekDay | tuesday, wednesday, thursday |
| localWeather | (pending local historical weather lookup in Aachen, Germany) |
| recordingTime | startTime field (hour of day, see recording details) |
| recordingFrameRate | 25 FPS |
| totalFrames | Varies per recording (= duration × 25) |
| duration | 441 – 1250 seconds |
| map | XX_background.png (one per recording) |
| laneRange | — |

**Recording Details** (24 recordings across 5 days):

| weekday | startTime | duration (min) | localWeather |
|---------|-----------|----------------|-------------|
| tuesday | 07:00 | 17 | — |
| wednesday | 11:00 | 18 | — |
| thursday | 09:00 | 123 | — |
| tuesday | 09:00 | 166 | — |
| wednesday | 09:00 | 73 | — |

The coordinate relationship between the provided trajectory position and the base map is shown in the figure.

![rounD Coordinate System](images/coord_rounD.png)

### 4.2 Intermediate Variables

| Variable | locationId 0 (Thiergarten) | locationId 1 (KackertstraBe) | locationId 2 (Neuweiler) |
|----------|---------------------------|-----------------------------|-----------------------|
| pix2meter | 98.43 | 67.52 | 73.35 |
| xUtmOrigin | 301221.3650 | 292669.4681 | 296309.7867 |
| yUtmOrigin | 5641501.3410 | 5630731.7040 | 5639851.9642 |
| latLocation | 50.8906 | 50.7906 | 50.8738 |
| lonLocation | 6.1747 | 6.0599 | 6.1066 |
| imgLon\*1–4, imgLat\*1–4 | (to be derived from UTM origin + image size × pix2meter) | — | — |


---

## 5. NGSIM

**Application Page**: https://data.transportation.gov/stories/s/Next-Generation-Simulation-NGSIM-Open-Data/i5zb-xe34/

**Data Source**: U.S. Department of Transportation (FHWA)

### 5.1 Meta Data

| Field | Value |
|-------|-------|
| datasetName | NGSIM |
| siteName | I-80, US-101, Lankershim Blvd, Peachtree St (Freeway / Urban arterial, USA) |
| recordingDate | See recording details below |
| weekDay | See recording details below |
| localWeather | Clear |
| recordingTime | See recording details below |
| recordingFrameRate | 10 FPS (I-80, US-101) / 15 FPS (Lankershim, Peachtree) |
| totalFrames | (no local NGSIM raw data files available) |
| duration | 15 minutes per segment |
| map | See map path below |
| laneRange | Lane_ID field available in data |

**Recording Details** (4 locations):

| Location | recordingDate | weekDay | recordingTime |
|----------|--------------|---------|---------------|
| I-80 | 2005-04-13 | Wednesday | 16:00–16:15, 17:00–17:15, 17:15–17:30 |
| US-101 | 2005-06-15 | Wednesday | 07:50–08:05, 08:05–08:20, 08:20–08:35 |
| Lankershim Blvd | 2005-06-16 | Thursday | 08:30–09:00 |
| Peachtree St | 2006-11-08 | Wednesday | 12:45–13:00, 16:00–16:15 |

The coordinate relationship between the provided trajectory position and the base map is shown in the figure.

![NGSIM Coordinate System](images/coord_NGSIM.png)

### 5.2 Intermediate Variables

| Variable | Value |
|----------|-------|
| pix2meter | — |
| imgLon\*1, imgLat\*1 | (NGSIM raw data uses Global_X/Y in feet, no map corner GPS coordinates) |
| imgLon\*2, imgLat\*2 | — |
| imgLon\*3, imgLat\*3 | — |
| imgLon\*4, imgLat\*4 | — |

