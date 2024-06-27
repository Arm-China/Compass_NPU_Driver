EM_LOG_TYPE_ERR = 1
EM_LOG_TYPE_ALT = 2
EM_LOG_TYPE_WAR = 4
EM_LOG_TYPE_INF = 8
EM_LOG_TYPE_DBG = 16
em_log_type_all = EM_LOG_TYPE_ERR | EM_LOG_TYPE_ALT | EM_LOG_TYPE_WAR | EM_LOG_TYPE_INF | EM_LOG_TYPE_DBG

class Log:
    m_log_level = em_log_type_all
    def __init__(self, level):
       self.m_log_level = level

    def error(self, log):
        if self.m_log_level & EM_LOG_TYPE_ERR:
            print(f"[Err] {log}")

    def alert(self, log):
        if self.m_log_level & EM_LOG_TYPE_ALT:
            print(f"[Alt] {log}")

    def warn(self, log):
        if self.m_log_level & EM_LOG_TYPE_WAR:
            print(f"[War] {log}")

    def info(self, log):
        if self.m_log_level & EM_LOG_TYPE_INF:
            print(f"[Inf] {log}")

    def debug(self, log):
        if self.m_log_level & EM_LOG_TYPE_DBG:
            print(f"[Dbg] {log}")
