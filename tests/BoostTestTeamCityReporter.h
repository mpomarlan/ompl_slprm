/* Copyright 2011 JetBrains s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Revision: 88625 $
 *
 * Files merged together and modified for use in OMPL
 * This code is only active when tests are executed in the TeamCity environment
 * AND when OMPL_TESTS_TEAMCITY is defined.
*/

#ifndef H_TEAMCITY_MESSAGES
#define H_TEAMCITY_MESSAGES

/** \brief Specify whether to enable unit test reporting to TeamCity */
#define OMPL_TESTS_TEAMCITY 0
#if OMPL_TESTS_TEAMCITY

#include <string>
#include <iostream>

namespace JetBrains {

std::string getFlowIdFromEnvironment();
bool underTeamcity();

class TeamcityMessages {
    std::ostream *m_out;

protected:
    std::string escape(std::string s);

    void openMsg(const std::string &name);
    void writeProperty(std::string name, std::string value);
    void closeMsg();

public:
    TeamcityMessages();

    void setOutput(std::ostream &);

    void suiteStarted(std::string name, std::string flowid = "");
    void suiteFinished(std::string name, std::string flowid = "");

    void testStarted(std::string name, std::string flowid = "");
    void testFailed(std::string name, std::string message, std::string details, std::string flowid = "");
    void testIgnored(std::string name, std::string message, std::string flowid = "");
    void testFinished(std::string name, int durationMs = -1, std::string flowid = "");
};

}

/* Copyright 2011 JetBrains s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Revision: 88625 $
*/

#include <stdlib.h>
#include <sstream>

namespace JetBrains {

std::string getFlowIdFromEnvironment() {
    const char *flowId = getenv("TEAMCITY_PROCESS_FLOW_ID");
    return flowId == NULL ? "" : flowId;
}

bool underTeamcity() {
    return getenv("TEAMCITY_PROJECT_NAME") != NULL;
}

TeamcityMessages::TeamcityMessages()
: m_out(&std::cout)
{}

void TeamcityMessages::setOutput(std::ostream &out) {
    m_out = &out;
}

std::string TeamcityMessages::escape(std::string s) {
    std::string result;

    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];

        switch (c) {
        case '\n': result.append("|n"); break;
        case '\r': result.append("|r"); break;
        case '\'': result.append("|'"); break;
        case '|':  result.append("||"); break;
        case ']':  result.append("|]"); break;
        default:   result.append(&c, 1);
        }
    }

    return result;
}

void TeamcityMessages::openMsg(const std::string &name) {
    // endl for http://jetbrains.net/tracker/issue/TW-4412
    *m_out << std::endl << "##teamcity[" << name;
}

void TeamcityMessages::closeMsg() {
    *m_out << "]";
    // endl for http://jetbrains.net/tracker/issue/TW-4412
    *m_out << std::endl;
    m_out->flush();
}

void TeamcityMessages::writeProperty(std::string name, std::string value) {
    *m_out << " " << name << "='" << escape(value) << "'";
}

void TeamcityMessages::suiteStarted(std::string name, std::string flowid) {
    openMsg("testSuiteStarted");
    writeProperty("name", name);
    if(flowid.length() > 0) {
        writeProperty("flowId", flowid);
    }

    closeMsg();
}

void TeamcityMessages::suiteFinished(std::string name, std::string flowid) {
    openMsg("testSuiteFinished");
    writeProperty("name", name);
    if(flowid.length() > 0) {
        writeProperty("flowId", flowid);
    }

    closeMsg();
}

void TeamcityMessages::testStarted(std::string name, std::string flowid) {
    openMsg("testStarted");
    writeProperty("name", name);
    if(flowid.length() > 0) {
        writeProperty("flowId", flowid);
    }

    closeMsg();
}

void TeamcityMessages::testFinished(std::string name, int durationMs, std::string flowid) {
    openMsg("testFinished");

    writeProperty("name", name);

    if(flowid.length() > 0) {
        writeProperty("flowId", flowid);
    }

    if(durationMs >= 0) {
        std::stringstream out;
        out << durationMs;
        writeProperty("duration", out.str());
    }

    closeMsg();
}

void TeamcityMessages::testFailed(std::string name, std::string message, std::string details, std::string flowid) {
    openMsg("testFailed");
    writeProperty("name", name);
    writeProperty("message", message);
    writeProperty("details", details);
    if(flowid.length() > 0) {
        writeProperty("flowId", flowid);
    }

    closeMsg();
}

void TeamcityMessages::testIgnored(std::string name, std::string message, std::string flowid) {
    openMsg("testIgnored");
    writeProperty("name", name);
    writeProperty("message", message);
    if(flowid.length() > 0) {
        writeProperty("flowId", flowid);
    }

    closeMsg();
}

}


/* Copyright 2011 JetBrains s.r.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Revision: 88625 $
*/

#include <sstream>

#include <boost/test/unit_test_suite_impl.hpp>
#include <boost/test/results_collector.hpp>
#include <boost/test/utils/basic_cstring/io.hpp>
#include <boost/test/unit_test_log.hpp>
#include <boost/test/unit_test_log_formatter.hpp>

using namespace boost::unit_test;

namespace JetBrains {

// Custom formatter for TeamCity messages
class TeamcityBoostLogFormatter: public boost::unit_test::unit_test_log_formatter {
    TeamcityMessages messages;
    std::string currentDetails;
    std::string flowId;

public:
    TeamcityBoostLogFormatter(const std::string &_flowId);
    TeamcityBoostLogFormatter();

    void log_start(std::ostream&, boost::unit_test::counter_t test_cases_amount);
    void log_finish(std::ostream&);
    void log_build_info(std::ostream&);

    void test_unit_start(std::ostream&, boost::unit_test::test_unit const& tu);
    void test_unit_finish(std::ostream&,
        boost::unit_test::test_unit const& tu,
        unsigned long elapsed);
    void test_unit_skipped(std::ostream&, boost::unit_test::test_unit const& tu);

    void log_exception(std::ostream&,
        boost::unit_test::log_checkpoint_data const&,
        boost::unit_test::const_string explanation);

    void log_entry_start(std::ostream&,
        boost::unit_test::log_entry_data const&,
        log_entry_types let);
    void log_entry_value(std::ostream&, boost::unit_test::const_string value);
    void log_entry_finish(std::ostream&);
};

// Fake fixture to register formatter
struct TeamcityFormatterRegistrar {
    TeamcityFormatterRegistrar() {
        if (JetBrains::underTeamcity()) {
            boost::unit_test::unit_test_log.set_formatter(new JetBrains::TeamcityBoostLogFormatter());
            boost::unit_test::unit_test_log.set_threshold_level(boost::unit_test::log_successful_tests);
        }
    }
};
BOOST_GLOBAL_FIXTURE(TeamcityFormatterRegistrar);

// Formatter implementation
std::string toString(const_string bstr) {
    std::stringstream ss;

    ss << bstr;

    return ss.str();
}

TeamcityBoostLogFormatter::TeamcityBoostLogFormatter(const std::string &_flowId)
: flowId(_flowId)
{}

TeamcityBoostLogFormatter::TeamcityBoostLogFormatter()
: flowId(getFlowIdFromEnvironment())
{}

void TeamcityBoostLogFormatter::log_start(std::ostream &out, counter_t test_cases_amount)
{}

void TeamcityBoostLogFormatter::log_finish(std::ostream &out)
{}

void TeamcityBoostLogFormatter::log_build_info(std::ostream &out)
{}

void TeamcityBoostLogFormatter::test_unit_start(std::ostream &out, test_unit const& tu) {
    messages.setOutput(out);

    if (tu.p_type == tut_case) {
        messages.testStarted(tu.p_name, flowId);
    } else {
        messages.suiteStarted(tu.p_name, flowId);
    }

    currentDetails.clear();
}

void TeamcityBoostLogFormatter::test_unit_finish(std::ostream &out, test_unit const& tu, unsigned long elapsed) {
    messages.setOutput(out);

    test_results const& tr = results_collector.results(tu.p_id);
    if (tu.p_type == tut_case) {
        if(!tr.passed()) {
            if(tr.p_skipped) {
                messages.testIgnored(tu.p_name, "ignored", flowId);
            } else if (tr.p_aborted) {
                messages.testFailed(tu.p_name, "aborted", currentDetails, flowId);
            } else {
                messages.testFailed(tu.p_name, "failed", currentDetails, flowId);
            }
        }

        messages.testFinished(tu.p_name, elapsed / 1000, flowId);
    } else {
        messages.suiteFinished(tu.p_name, flowId);
    }
}

void TeamcityBoostLogFormatter::test_unit_skipped(std::ostream &out, test_unit const& tu)
{}

void TeamcityBoostLogFormatter::log_exception(std::ostream &out, log_checkpoint_data const&, const_string explanation) {
    std::string what = toString(explanation);

    out << what << std::endl;
    currentDetails += what + "\n";
}

void TeamcityBoostLogFormatter::log_entry_start(std::ostream&, log_entry_data const&, log_entry_types let)
{}

void TeamcityBoostLogFormatter::log_entry_value(std::ostream &out, const_string value) {
    out << value;
    currentDetails += toString(value);
}

void TeamcityBoostLogFormatter::log_entry_finish(std::ostream &out) {
    out << std::endl;
    currentDetails += "\n";
}

}

#endif /* OMPL_TESTS_TEAMCITY */

#endif /* H_TEAMCITY_MESSAGES */
