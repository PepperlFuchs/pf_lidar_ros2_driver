#include "pf_driver/pf/http_helpers/http_interface.h"
#include "pf_driver/pf/http_helpers/curl_resource.h"
#include "pf_driver/pf/http_helpers/http_helpers.h"

HTTPInterface::HTTPInterface(std::string host, std::string path) : host(std::move(host)), base_path(std::move(path))
{
}

const Json::Value HTTPInterface::get(const std::string& command,
    const std::initializer_list<param_type>& list)
{
  CurlResource res(host);
  res.append_path(base_path);
  res.append_path(command);
  res.append_query(list);
  return get_(res);
}

const Json::Value HTTPInterface::get(const std::string& command,
    const param_map_type& params)
{
  CurlResource res(host);
  res.append_path(base_path);
  res.append_path(command);
  res.append_query(params);
  return get_(res);
}

const Json::Value HTTPInterface::get_(CurlResource& res)
{
  Json::Value json_resp;

  try
  {
    res.get(json_resp);
  }
  catch (curlpp::RuntimeError& e)
  {
    json_resp["error_code"] = -1;
    json_resp["error_text"] = std::string(e.what());
  }
  catch (curlpp::LogicError& e)
  {
    json_resp["error_code"] = -1;
    json_resp["error_text"] = std::string(e.what());
  }

  return json_resp;
}
