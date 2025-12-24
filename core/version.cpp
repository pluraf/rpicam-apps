/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * AUTO-GENERATED, DO NOT MODIFY!
 */
#include <string>

#include "core/version.hpp"
#include "encoder/encoder.hpp"
#include "preview/preview.hpp"

extern "C" {

const char *RPiCamAppsVersion()
{
	static const std::string version {"v1.10.0 73e61a44d032-dirty 13-11-2025 (14:55:57)"};
	return version.c_str();
}

const char *RPiCamAppsCapabilities(const std::string &preview_libs, const std::string &encoder_libs)
{
	auto &p = PreviewFactory::GetInstance();
	p.LoadPreviewLibraries(preview_libs);

	auto &e = EncoderFactory::GetInstance();
	e.LoadEncoderLibraries(encoder_libs);

	static const std::string caps = 
		"egl:" + std::to_string(p.HasPreview("egl")) + 
		" qt:" + std::to_string(p.HasPreview("qt")) +
		" drm:" + std::to_string(p.HasPreview("drm")) +
		" libav:" + std::to_string(e.HasEncoder("libav"));

	return caps.c_str();
}

}
