// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>
#include "parser_test.h"
#include "aipu.h"

TEST_CASE_FIXTURE(ParserTest, "get_graph_bin_version")
{
    uint32_t g_version = 0;

    g_version = m_parser->get_graph_bin_version(gbin);
    CHECK(g_version != 0);
}

TEST_CASE_FIXTURE(ParserTest, "parse_graph")
{
    aipu_status_t ret;

    ret = m_parser->parse_graph(gbin, 0, *(dynamic_cast<Graph*>(p_gobj)));
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_GBIN);

    ret = m_parser->parse_graph(gbin, fsize, *(dynamic_cast<Graph*>(p_gobj)));
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

#if (defined ZHOUYI_V12)
TEST_CASE_FIXTURE(ParserTest, "parse_graph_header_top")
{
    aipu_status_t ret;

    gbin.seekg(0, gbin.beg);

    ret = m_parser->parse_graph_header_top(gbin, fsize, *(dynamic_cast<Graph*>(p_gobj)));
    CHECK(ret == AIPU_STATUS_SUCCESS);
}
#endif
