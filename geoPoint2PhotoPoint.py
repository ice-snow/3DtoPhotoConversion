import numpy as np
import math
import json
import urllib.request


def CalcInMatrix(imageSize=[1134, 756], senseSize=[40.0, 26.7], ff=3.5):
    du = senseSize[0] / imageSize[0]
    dv = senseSize[1] / imageSize[1]
    fu = ff / du
    fv = ff / dv
    v0 = imageSize[1] / 2
    u0 = imageSize[0] / 2
    inMatrix = np.array([[fu, 0, u0], [0, fv, v0], [0, 0, 1]])
    return inMatrix


def CalcRotationMatrix2(vectorBefor, vectorAfter):
    crossProduct = [0 for col in range(3)]
    crossProduct[0] = vectorBefor[1] * \
        vectorAfter[2] - vectorBefor[2] * vectorAfter[1]
    crossProduct[1] = vectorBefor[2] * \
        vectorAfter[0] - vectorBefor[0] * vectorAfter[2]
    crossProduct[2] = vectorBefor[0] * \
        vectorAfter[1] - vectorBefor[1] * vectorAfter[0]
    u = np.array(crossProduct)

    dotProduct = np.dot(vectorBefor, vectorAfter)
    nomalBefor = math.sqrt(np.dot(vectorBefor, vectorBefor))
    nomalAfter = math.sqrt(np.dot(vectorAfter, vectorAfter))

    rotationAngle = math.acos(dotProduct / nomalBefor / nomalAfter)
    u = u / math.sqrt(np.dot(u, u))

    rotatinMatrix = [[0 for col in range(3)] for row in range(3)]
    rotatinMatrix[0][0] = math.cos(
        rotationAngle) + u[0] * u[0] * (1 - math.cos(rotationAngle))
    rotatinMatrix[1][0] = u[0] * u[1] * \
        (1 - math.cos(rotationAngle) - u[2] * math.sin(rotationAngle))
    rotatinMatrix[2][0] = u[1] * \
        math.sin(rotationAngle) + u[0] * u[2] * (1 - math.cos(rotationAngle))

    rotatinMatrix[0][1] = u[2] * \
        math.sin(rotationAngle) + u[0] * u[1] * (1 - math.cos(rotationAngle))
    rotatinMatrix[1][1] = math.cos(
        rotationAngle) + u[1] * u[1] * (1 - math.cos(rotationAngle))
    rotatinMatrix[2][1] = -u[0] * \
        math.sin(rotationAngle) + u[1] * u[2] * (1 - math.cos(rotationAngle))

    rotatinMatrix[0][2] = -u[1] * \
        math.sin(rotationAngle) + u[0] * u[2] * (1 - math.cos(rotationAngle))
    rotatinMatrix[1][2] = u[0] * \
        math.sin(rotationAngle) + u[1] * u[2] * (1 - math.cos(rotationAngle))
    rotatinMatrix[2][2] = math.cos(
        rotationAngle) + u[2] * u[2] * (1 - math.cos(rotationAngle))

    rotatinMatrixEx = np.array(rotatinMatrix)
    return rotatinMatrixEx


def CalcRotationMatrix1(rotationAngle):
    rotatinMatrix = [[0 for col in range(3)] for row in range(3)]
    rotatinMatrix[0][0] = math.cos(
        rotationAngle[0]) * math.cos(rotationAngle[1])
    rotatinMatrix[1][0] = math.sin(
        rotationAngle[0]) * math.cos(rotationAngle[1])
    rotatinMatrix[2][0] = -math.sin(rotationAngle[1])

    rotatinMatrix[0][1] = math.cos(rotationAngle[0]) * math.sin(rotationAngle[1]) * math.sin(
        rotationAngle[2]) - math.sin(rotationAngle[0]) * math.cos(rotationAngle[2])
    rotatinMatrix[1][1] = math.sin(rotationAngle[0]) * math.sin(rotationAngle[1]) * math.sin(
        rotationAngle[2]) + math.cos(rotationAngle[0]) * math.cos(rotationAngle[2])
    rotatinMatrix[2][1] = math.cos(
        rotationAngle[1]) * math.sin(rotationAngle[2])

    rotatinMatrix[0][2] = math.cos(rotationAngle[0]) * math.sin(rotationAngle[1]) * math.cos(
        rotationAngle[2]) - math.sin(rotationAngle[0]) * math.sin(rotationAngle[2])
    rotatinMatrix[1][2] = math.sin(rotationAngle[0]) * math.sin(rotationAngle[1]) * math.cos(
        rotationAngle[2]) + math.cos(rotationAngle[0]) * math.sin(rotationAngle[2])
    rotatinMatrix[2][2] = math.cos(
        rotationAngle[1]) * math.cos(rotationAngle[2])
    res = np.array(rotatinMatrix)
    return res


def OuterMatrix(cameraH):
    ''' pi = math.pi
    rotationAngle = np.array([50 * pi / 180, 50 * pi / 180, 50 * pi / 180])
    rotationMatrix = CalcRotationMatrix1(rotationAngle) '''
    carVect = np.array([1, 1, 1])
    cdrVect = np.array([1, 1, 0])
    rotationMatrix = CalcRotationMatrix2(carVect, cdrVect)

    translationVector = np.array([0, 0, cameraH])
    # t1 = np.row_stack((rotationMatrix, np.array([0, 0, 0])))
    outerMatrix = np.column_stack(
        (rotationMatrix, np.transpose(translationVector)))
    return outerMatrix


def GetPosition(centerLon, cameraPosition, geoPosition):
    relatPoint = ChangeXYZ(centerLon, cameraPosition, geoPosition)

    inM = CalcInMatrix()

    # inMEx = np.column_stack((inM, np.transpose(np.array([0, 0, 0]))))
    outM = OuterMatrix(relatPoint[2])
    position = np.transpose(
        np.array([relatPoint[0], relatPoint[1], geoPosition[2], 1]))
    res = np.dot(np.dot(inM, outM), position)
    return res


def ChangeXYZ(centerLon, cameraPosition, geoPosition):
    cameraPo = LonLat2UTM(centerLon, [cameraPosition[0], cameraPosition[1]])
    pointPo = LonLat2UTM(centerLon, [geoPosition[0], geoPosition[1]])

    ppo = [pointPo[0] - cameraPo[0], pointPo[1] -
           cameraPo[1], cameraPosition[2]]

    return ppo


def LonLat2UTM(centerLon, position):
    c, e, f = centerLon, position[1], position[0]
    d = math.floor(c) + (math.floor(c * 100) - math.floor(c) * 100) / \
        60 + (c * 10000 - math.floor(c * 100) * 100) / 3600
    g = math.floor(e) + (math.floor(e * 100) - math.floor(e) * 100) / \
        60 + (e * 10000 - math.floor(e * 100) * 100) / 3600
    h = math.floor(f) + (math.floor(f * 100) - math.floor(f) * 100) / \
        60 + (f * 10000 - math.floor(f * 100) * 100) / 3600
    i = h - d
    j = i / 57.2957795130823
    k = math.tan(Radians(g))
    l = math.cos(Radians(g))
    m = 0.006738525415 * l * l
    n = k * k
    o = 1 + m
    p = 6399698.9018 / math.sqrt(o)
    q = j * j * l * l
    r = k * l
    s = r * r
    t = (32005.78006 + s * (133.92133 + s * 0.7031))
    utmX = 6367558.49686 * g / 57.29577951308 - r * l * t + \
        ((((n - 58) * n + 61) * q / 30 + (4 * m + 5) * o - n)
         * q / 12 + 1) * p * k * q / 2
    utmY = ((((n - 18) * n - (58 * n - 14) * m + 5) *
             q / 20 + o - n) * q / 6 + 1) * p * (j * l)
    return [utmX, utmY]


def Radians(angle):
    return angle * math.pi / 180


def GenUrl(po1, po2, level):
    url = "http://47.92.3.2:800/geoSOT-API/Range2Envelops/{0}/{1}/{2}/{3}/{4}".format(
        po1[0], po1[1], po2[0], po2[1], level)
    return url


def main(rangeJson):
    centerLog = 117.0
    cameraPosition = [116.317317, 39.96533, 4]
    cameraJson = []
    for rj in rangeJson:
        code = rj['code']
        pos = rj['range']
        rec = []
        for po in pos:
            po22 = GetPosition(centerLog, cameraPosition, po)
            po22 = po22 / po22[2]
            rec.append(list(np.round(po22)))

        recJson = {
            'code': code,
            'range': rec
        }
        print(recJson)
        cameraJson.append(recJson)
    # recJsons = json.dumps(cameraPosition)
    with open('rangeJson.json', 'w') as f:
        json.dump(cameraJson, f)


def run():
    po1 = [116.317416, 39.965014]
    po2 = [116.31843, 39.966012]
    level = 23
    high = 2
    url = GenUrl(po1, po2, level)
    print(url)
    geoPoints = json.load(urllib.request.urlopen(url))
    rangeJson = []
    envelops = geoPoints['envelops']
    for item in envelops:
        code = item['code']
        rg = item['range']
        rectangle = [[rg['minLon'], rg['minLat'], high], [rg['minLon'], rg['maxLat'], high], [
            rg['maxLon'], rg['maxLat'], high],  [rg['maxLon'], rg['minLat'], high]]
        itemRange = {
            'code': code,
            'range': rectangle
        }
        rangeJson.append(itemRange)
    # geoPoints = [[116.31733, 39.96533, 2], [116.31733, 39.96534, 2], [116.31753, 39.96531, 2]]
    main(rangeJson)


if(__name__ == '__main__'):
    run()
