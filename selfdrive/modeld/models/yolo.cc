#include "selfdrive/modeld/models/yolo.h"

using namespace json11;

Json classes = Json::array { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush" };

class Vec4f {
public:
    Vec4f() = default;
    Vec4f(float _x, float _y, float _z, float _w) {
        data[0] = _x;
        data[1] = _y;
        data[2] = _z;
        data[3] = _w;
    }

    float data[4] = { 0, 0, 0, 0 };

    inline float operator [](int i) const { return data[i]; }
    inline float& operator [](int i) { return data[i]; }
};

class Rect {
public:
    Rect() = default;
    Rect(float _x, float _y, float _width, float _height) : x(_x), y(_y),   width(_width),  height(_height) {}

    float x      = 0; //!< x coordinate of the top-left corner
    float y      = 0; //!< y coordinate of the top-left corner
    float width  = 0; //!< width of the rectangle
    float height = 0; //!< height of the rectangle

    const float area() const {
        return width * height;
    }
};

inline Rect& operator&=(Rect& lhs, const Rect& rhs) {
    int x1 = std::max(lhs.x, rhs.x);
    int y1 = std::max(lhs.y, rhs.y);
    lhs.width  = std::min(lhs.x + lhs.width,  rhs.x + rhs.width) -  x1;
    lhs.height = std::min(lhs.y + lhs.height, rhs.y + rhs.height) - y1;
    lhs.x = x1;
    lhs.y = y1;
    if( lhs.width <= 0 || lhs.height <= 0 )
        lhs = Rect();
    return lhs;
}

inline Rect operator&(const Rect& lhs, const Rect& rhs) {
    Rect result = lhs;
    return result &= rhs;
}


void nms(const std::vector<Rect> &srcRects, std::vector<Rect> &resRects, std::vector<int> &resIndexs, float thresh) {
    resRects.clear();
    const size_t size = srcRects.size();
    if (!size) return;
    // Sort the bounding boxes by the bottom - right y - coordinate of the bounding box
    std::multimap<int, size_t> idxs;
    for (size_t i = 0; i < size; ++i){
        idxs.insert(std::pair<int, size_t>(srcRects[i].y + srcRects[i].height, i));
    }
    // keep looping while some indexes still remain in the indexes list
    while (idxs.size() > 0){
        // grab the last rectangle
        auto lastElem = --std::end(idxs);
        const Rect& last = srcRects[lastElem->second];
        resIndexs.push_back(lastElem->second);
        resRects.push_back(last);
        idxs.erase(lastElem);
        for (auto pos = std::begin(idxs); pos != std::end(idxs); ){
            // grab the current rectangle
            const Rect& current = srcRects[pos->second];
            float intArea = (last & current).area();
            float unionArea = last.area() + current.area() - intArea;
            float overlap = intArea / unionArea;
            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > thresh) pos = idxs.erase(pos);
            else ++pos;
        }
    }
}

std::string parse_yolo_outputs(float *output) {
    size_t size = 16380 * 85;
    int dimensions = 85; // 0,1,2,3 ->box,4->confidence，5-85 -> coco classes confidence
    int rows = size / dimensions; //25200
    int confidenceIndex = 4;
    int labelStartIndex = 5;
    float modelWidth = 640.0;
    float modelHeight = 640.0;
    float xGain = modelWidth / 640;
    float yGain = modelHeight / 416;

    std::vector<Vec4f> locations;
    std::vector<int> labels;
    std::vector<float> confidences;

    std::vector<Rect> src_rects;
    std::vector<Rect> res_rects;
    std::vector<int> res_indexs;

    Rect rect;
    Vec4f location;
    for (int i = 0; i < rows; ++i) {
        int index = i * dimensions;
        if (output[index+confidenceIndex] <= 0.4f) continue;

        for (int j = labelStartIndex; j < dimensions; ++j) {
            output[index+j] = output[index+j] * output[index+confidenceIndex];
        }

        for (int k = labelStartIndex; k < dimensions; ++k) {
            if (output[index+k] <= 0.5f) continue;

            location[0] = (output[index] - output[index+2] / 2) / xGain; // top left x
            location[1] = (output[index + 1] - output[index+3] / 2) / yGain; // top left y
            location[2] = (output[index] + output[index+2] / 2) / xGain; // bottom right x
            location[3] = (output[index + 1] + output[index+3] / 2) / yGain; // bottom right y

            locations.emplace_back(location);

            rect = Rect(location[0], location[1], location[2] - location[0], location[3] - location[1]);
            src_rects.push_back(rect);
            labels.emplace_back(k-labelStartIndex);

            confidences.emplace_back(output[index+k]);
        }

    }

    nms(src_rects, res_rects, res_indexs, 0.5);

    std::vector<Json> items;
    for (int i = 0; i < res_indexs.size(); ++i) {
        int index = res_indexs[i];
        Json item = Json::object {
            { "label", classes[labels[index]] },
            { "score", confidences[index] },
            { "location", Json::object {
                { "x", locations[index][0] },
                { "y", locations[index][1] },
                { "width", locations[index][2] - locations[index][0] },
                { "height", locations[index][3] - locations[index][1] }
            }}
        };
        items.push_back(item);
    }

    Json result = Json::object {
        { "code",  0 },
        { "msg", "success" },
        { "data", Json::array { items }}
    };

    return result.dump();
}
