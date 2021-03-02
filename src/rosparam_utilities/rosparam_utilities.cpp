#include <rosparam_utilities/rosparam_utilities.h>


namespace rosparam_utilities 
{


double toDouble(const XmlRpc::XmlRpcValue& node, const std::string& field, const std::string& log)
{
  return fromXmlRpcValue<double>(node, field, log );
}
int toInt(const XmlRpc::XmlRpcValue& node, const std::string& field, const std::string& log)
{
  return fromXmlRpcValue<int>(node, field, log );
}
bool toBool(const XmlRpc::XmlRpcValue& node, const std::string& field, const std::string& log)
{
  return fromXmlRpcValue<bool>(node, field, log );
}
std::string toString(const XmlRpc::XmlRpcValue& node, const std::string& field, const std::string& log)
{
  return fromXmlRpcValue<std::string>( node, field, log );
}
  


void toXmlRpcValue( const int& t,  XmlRpc::XmlRpcValue& xml_value, const std::string& format )
{
  if( format == "dec" )
  {
    xml_value =( int ) t;
  }
  else if( format == "hex" )
  {
    std::stringstream s;
    s << "0x" << std::hex << t ;
    xml_value = s.str();
  }
  else if( format == "bin" )
  {
    std::stringstream s;
    s << "0b" << std::bitset<32>( t );
    xml_value = s.str();
  }
}
void toXmlRpcValue( const std::string& t,  XmlRpc::XmlRpcValue& xml_value )
{
  xml_value =( std::string ) t;
}
void toXmlRpcValue( const bool& t,  XmlRpc::XmlRpcValue& xml_value )
{
  xml_value =( bool ) t;
}
void toXmlRpcValue( const double& t,  XmlRpc::XmlRpcValue& xml_value )
{
  xml_value =( double ) t;
}



bool check( const XmlRpc::XmlRpcValue& config, const std::vector< std::string >& required_fields )
{
  for( std::vector< std::string >::const_iterator it = required_fields.begin(); it != required_fields.end(); it++ )
  {
    if( !config.hasMember( *it ) )
    {
      ROS_ERROR( " CHECK XmlRpcValue -The field '%s' does not exist. ", it->c_str() );
      return false;
    }
  }

  return true;
}


}
