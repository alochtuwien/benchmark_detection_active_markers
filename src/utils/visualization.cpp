//
// Created by aloch on 19.12.22.
//

#include "../../include/utils/visualization.hpp"

template<typename type>
std::vector<type> unique(cv::Mat in) {
    assert(in.channels() == 1 && "This implementation is only for single-channel images");
    auto begin = in.begin<type>(), end = in.end<type>();
    auto last = std::unique(begin, end);    // remove adjacent duplicates to reduce size
    std::sort(begin, last);                 // sort remaining elements
    last = std::unique(begin, last);        // remove duplicates
    return std::vector<type>(begin, last);
}

VisualizationController::VisualizationController(int width_new, int height_new, Camera *cam, MarkersManager *markers) {
    width = width_new;
    height = height_new;

    cam_obj = cam;
    marker_manager = markers;


    ui_ptr = new Metavision::MTWindow("Active markers", width, height*2, Metavision::Window::RenderMode::BGR);
//    prod = new VisualizationProducer(width, height);
    ui_ptr->set_keyboard_callback([this](Metavision::UIKeyEvent key, int scancode, Metavision::UIAction action, int mods) {
        if (action == Metavision::UIAction::RELEASE && key == Metavision::UIKeyEvent::KEY_ESCAPE){
            to_close = true;
            return;
        }
        if (action == Metavision::UIAction::RELEASE && key == Metavision::UIKeyEvent::KEY_P){
            std::cout << "PAUSE" << std::endl;
            return;
        }

    });
    img = cv::Mat::zeros(height, width, CV_8UC1);
    output = cv::Mat::zeros(height, width, CV_8UC3);
    labels = cv::Mat::zeros(height, width, CV_8UC1);
    counts = cv::Mat::zeros(height, width, CV_8UC1);

    to_close = false;
    closed = false;

    period = (long long)1000000 / fps;
    accumulation_period = (long long)1000000 / accumulation_fps;
    cutoff = min_freq / accumulation_period;

}

[[noreturn]] void VisualizationController::runtimeLoop() {
    int i = 0;
    bool to_check = false;
    int size_hist[3] = { height, width, 20};
    cv::Mat tses(3, size_hist, CV_32S, cv::Scalar::all(0));

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char buffer [80];
    strftime (buffer,80,"%Y-%m-%d-%H-%M-%S.csv",now);
    csvfile csv(buffer);

    csv << "TS" << "ID" << "X" << "Y" << "Z" << "R1" << "R2" << "R3" << endrow;

    while(!to_close.load()){


        if (buffers.getBatch()){
            if (next_output_required_at == 0){
                next_output_required_at = buffers.current_batch.first->front()->t + period;
                std::cout << "First ts " << next_output_required_at << std::endl;
            }
            if (next_output_required_at < buffers.current_batch.first->back()->t){
                to_check = true;
            }

            if (buffers.current_batch.first->back()->t > (next_output_required_at - accumulation_period))
            {
                for (const Metavision::Event2d* ev : *buffers.current_batch.first) {
                    if (ev->t > (next_output_required_at - accumulation_period)) {
                        img.at<uint8_t>(ev->y, ev->x) = 255;
                        if (counts.at<uint8_t>(ev->y, ev->x) < 20){
                            tses.at<int>(ev->y, ev->x, (counts.at<uint8_t>(ev->y, ev->x))) = ev->t;
                        }
                        counts.at<uint8_t>(ev->y, ev->x) += 1;

                        if (ev->t > next_output_required_at) {
                            next_output_required_at += period;

                            cv::Mat stats;
                            cv::Mat centroids;
                            std::vector<std::vector<cv::Point>> contours;
                            std::vector<std::vector<cv::Point>> filtered_contours;
                            std::vector<cv::Vec4i> hierarchy;




                            auto start = std::chrono::high_resolution_clock::now();

                            cv::Mat counts_org = counts.clone();
                            cv::threshold(counts, counts,  cutoff, 255,  cv::THRESH_BINARY);

                            cv::findContours(counts, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                            std::vector<cv::Rect> boundRect;
                            std::vector<int> areas;
                            std::vector<float> frequencies;
                            float min_area = 5;
                            for (unsigned int i = 0; i < contours.size(); i++) {
                                float area = cv::contourArea(contours[i]);

                                if(min_area < area){
                                    filtered_contours.push_back(contours[i]);
                                    boundRect.push_back(boundingRect( contours[i] ));
                                    areas.push_back(area);
                                }
                            }

                            for (unsigned int i = 0; i < boundRect.size(); i++) {

                                std::vector<int> diffs(201, 0);

                                for (unsigned int x = boundRect[i].x; x <= boundRect[i].x + boundRect[i].width; ++x){
                                    for (unsigned int y = boundRect[i].y; y <= boundRect[i].y + boundRect[i].height; ++y){

                                        for (unsigned int channel = 0; channel < std::max(int(counts_org.at<uint8_t>(y, x)) - 1, 0); ++channel){
                                            diffs[std::min(tses.at<int>(y, x, channel+1) - tses.at<int>(y, x, channel), 201)] += 1;
                                        }
                                    }
                                }
                                diffs.back() = 0;

                                auto it = std::max_element(std::begin(diffs), std::end(diffs));
                                auto period_mode = std::distance(std::begin(diffs), it);
                                if (period_mode > 0){
                                    float sum = float((period_mode * diffs[period_mode] + ((period_mode-1) * diffs[period_mode-1]) + ((period_mode+1) * diffs[period_mode+1])));
                                    float cnt = float(diffs[period_mode] + diffs[period_mode + 1] + diffs[period_mode - 1]);
                                    auto freq = float(sum / cnt);
                                    frequencies.push_back(round(float(1000000 / freq)/200) * 200);
                                    std::cout << frequencies[i] << " ";
                                }
                                else{
                                    frequencies.push_back(float(0.0));
                                }

                            }
                            std::cout << std::endl;

                            auto found = marker_manager->matchMarkers(frequencies, filtered_contours);

                            auto stop = std::chrono::high_resolution_clock::now();
                            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                            std::cout << "Time taken by function: "
                                 << duration.count() << " microseconds and found " << filtered_contours.size() << "Blobs" << std::endl;


                            getOutput();

                            for (unsigned int i = 0; i < boundRect.size(); i++) {
                                cv::rectangle(counts, boundRect[i], cv::Scalar(0, 0, 255), 2);
                            }
                            for (auto marker: found){
                                cv::circle(counts, marker.projected_centerpoint, 1, cv::Scalar(255, 0, 0), 5);
                                csv << ev->t << marker.id << marker.t_vec.at<double>(0) << marker.t_vec.at<double>(1) << marker.t_vec.at<double>(2);
                                csv << marker.r_vec.at<double>(0) << marker.r_vec.at<double>(1) << marker.r_vec.at<double>(2);

                                csv << endrow;
                            }
                            cv::Mat matF;
                            matF.push_back(output);
                            matF.push_back(counts);
                            ui_ptr->show_async(matF);
                            Metavision::EventLoop::poll_and_dispatch();

                            resetState();
                        }

                    }
                }
            }
        }
        else {
            ui_ptr->set_close_flag();
            closed = true;
            to_close = true;
        }
    }
    ui_ptr->set_close_flag();
    usleep(100);
    closed = true;

}

void VisualizationController::start() {
    if(buffers.input_buffer != nullptr){
        thread_ptr = new boost::thread(&VisualizationController::runtimeLoop, this);
    }
}


void VisualizationController::getOutput() {
    cv::cvtColor(img, output, cv::COLOR_GRAY2BGR);
//    cv::normalize(counts, counts, 0, 255, cv::NORM_MINMAX);
    cv::cvtColor(counts, counts, cv::COLOR_GRAY2BGR);
//    cv::cvtColor(labels, counts, cv::COLOR_GRAY2BGR);

}

void VisualizationController::resetState(){
    img = cv::Mat::zeros(height, width, CV_8UC1);
    output = cv::Mat::zeros(height, width, CV_8UC3);
    counts = cv::Mat::zeros(height, width, CV_8UC1);
    labels = cv::Mat::zeros(height, width, CV_16UC1);
}

bool VisualizationController::isFinished() {
    return closed.load();
}




