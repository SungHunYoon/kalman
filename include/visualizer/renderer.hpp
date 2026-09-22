#pragma once

#include "visualizer/view_model.hpp"

class Renderer {
public:
	Renderer(int width = 1440, int height = 900);
	~Renderer();
	Renderer(const Renderer&) = delete;
	Renderer& operator=(const Renderer&) = delete;

	bool should_close() const;
	void update_controls(ViewerModel& model);
	void draw(const ViewerModel& model);

private:
	struct Impl;
	Impl* impl_;
	bool paused_ = false;
	bool follow_ = true;
	bool show_gps_ = true;
	bool show_covariance_ = true;
};
