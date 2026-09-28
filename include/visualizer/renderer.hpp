#pragma once

#include "visualizer/view_model.hpp"
#include "visualizer/view_geometry.hpp"

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
	ViewMode mode_ = ViewMode::Overview;
	bool show_gps_ = false;
	bool show_covariance_ = false;
};
