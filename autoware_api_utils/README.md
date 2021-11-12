# autoware_api_utils

## Overview

This package provides utility functions for Autoware API.

## Description

The functions provided are as follows.

### Logging for Service Call

By using the Service and Client classes in autoware_api_utils instead of rclcpp, it logs when the service is called on the server and when the service is called on the client.

```cpp
autoware_api_utils::ServiceProxyNodeInterface proxy(this);
srv = proxy.create_service<ServiceType>(...);
cli = proxy.create_client<ServiceType>(...);
```

### Synchronous Service Call

Synchronous service calls are supported that block processing until the response is returned. It checks service existence and timeout and returns the result as type ResponseStatus.

```cpp
auto [status, res] = cli_->call(req);
if (!autoware_api_utils::is_success(status)) {
  response->status = status;
  return;
}
response->status = autoware_api_utils::response_success(resp->message);
```
