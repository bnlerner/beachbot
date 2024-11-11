from log import logger

_logger = logger.Logger()

__builtins__["debug"] = _logger.debug
debug = _logger.debug
data = _logger.data
info = _logger.info
error = _logger.error
warning = _logger.warning
