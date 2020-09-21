// This file provides DebugLogDecoder "class"

var DebugLogDecoder = function(modules, globalEvents, functionIdToNameMap)
{
    this.modules = modules;
    this.globalEvents = globalEvents;
    this.functionIdToNameMap = functionIdToNameMap;
}

DebugLogDecoder.prototype.createUnknownModule = function(moduleId)
{
    var module = {};
    module.name = "Unknown (" + moduleId + ")";
    module.moduleId = moduleId;
    return module;
}

DebugLogDecoder.prototype.createUnknownLocalEvent = function(eventId)
{
    var result = {};
    result.eventId = eventId ;
    result.text = "Unknown local event (" + eventId + ")";
    return result;
}

DebugLogDecoder.prototype.createUnknownGlobalEvent = function(eventId)
{
    var result = {};
    result.eventId = eventId ;
    result.text = "Unknown global event (" + eventId + ")";
    return result;
}

DebugLogDecoder.prototype.createUnknownFunctionObj = function(functionId)
{
    var result = {};
    result.id = functionId ;
    result.name = "Unknown function (" + functionId + ")";
    return result;
}

DebugLogDecoder.prototype.searchArrayObjectsByPropertyId = function(arr, id)
{
    if (!Array.isArray(arr)) {
        return null;
    }
    var result = arr.find(obj => obj.id == id);
    if (result === undefined) {
        result = null;
    }
    return result;
}

DebugLogDecoder.prototype.enumValueToText = function(arr, value)
{
    if (!Array.isArray(arr)) {
        return "Unknown enum (" + value + ")";
    }
    
    var result = arr.find(obj => obj.value == value);
    if (result === undefined) {
        result = "Unknown enum (" + value + ")";
    }
    else {
        result = result.text;
    }
    return result;
}

DebugLogDecoder.prototype.getModuleById = function(moduleId)
{
    var module = this.searchArrayObjectsByPropertyId(this.modules, moduleId);
    if (module == null) 
    {
        module = this.createUnknownModule(moduleId);
    }
    return module;
}

DebugLogDecoder.prototype.getLocalEventById = function(module, eventId)
{
    var result = this.searchArrayObjectsByPropertyId(module.localEvents, eventId);
    if (result == null)
    {
        result = this.createUnknownLocalEvent(eventId);
    }
    return result;
}

DebugLogDecoder.prototype.getGlobalEventById = function(eventId)
{
    var result = this.searchArrayObjectsByPropertyId(this.globalEvents, eventId);
    if (result == null)
    {
        result = this.createUnknownGlobalEvent(eventId);
    }
    return result;
}

DebugLogDecoder.prototype.getEventText = function(event, eventParam)
{
    var result = "" ;
    result += event.text ;
    if (event.paramType != null) 
    {
        result += " ";
        switch (event.paramType) 
        {
            case "uint":
                result += eventParam.toString();
                break;
            case "enum":
                result += this.enumValueToText(event.enumValues, eventParam);
                break;
            default:
                // Possibly you have typo or introduced new paramType value not handled here
                throw "Internal error, unknown event.paramType";
        }
    }
    return result;
}

DebugLogDecoder.prototype.getFunctionObjById = function(functionId)
{
    var result = this.searchArrayObjectsByPropertyId(this.functionIdToNameMap, functionId);
    if (result == null) {
        result = this.createUnknownFunctionObj(functionId);
    }
    return result;
}

DebugLogDecoder.prototype.parseEventCode = function(eventCode)
{
    var result = {};
    
    // Constants related to log word encoding
    const NRF_802154_DEBUG_LOG_TYPE_BITPOS = 28;
    const NRF_802154_DEBUG_LOG_TYPE_BITMASK = 0xF;
    const NRF_802154_DEBUG_LOG_MODULE_ID_BITPOS = 22;
    const NRF_802154_DEBUG_LOG_MODULE_ID_BITMASK = 0x3F;
    const NRF_802154_DEBUG_LOG_EVENT_ID_BITPOS = 16;
    const NRF_802154_DEBUG_LOG_EVENT_ID_BITMASK = 0x3F;
    const NRF_802154_DEBUG_LOG_EVENT_PARAM_BITPOS = 0;
    const NRF_802154_DEBUG_LOG_EVENT_PARAM_BITMASK = 0xFFFF;
    const NRF_802154_DEBUG_LOG_FUNCTION_ID_BITPOS = 0;
    const NRF_802154_DEBUG_LOG_FUNCTION_ID_BITMASK = 0x3FFFFF;

    const NRF_802154_LOG_TYPE_FUNCTION_ENTER = 1;
    const NRF_802154_LOG_TYPE_FUNCTION_EXIT  = 2;
    const NRF_802154_LOG_TYPE_LOCAL_EVENT    = 3;
    const NRF_802154_LOG_TYPE_GLOBAL_EVENT   = 4;
    
    
    result.eventType = (eventCode >> NRF_802154_DEBUG_LOG_TYPE_BITPOS) & NRF_802154_DEBUG_LOG_TYPE_BITMASK;
    switch(result.eventType)
    {
        case NRF_802154_LOG_TYPE_FUNCTION_ENTER:
            result.module = this.getModuleById((eventCode >> NRF_802154_DEBUG_LOG_MODULE_ID_BITPOS) & NRF_802154_DEBUG_LOG_MODULE_ID_BITMASK);            
            result.functionId = (eventCode >> NRF_802154_DEBUG_LOG_FUNCTION_ID_BITPOS) & NRF_802154_DEBUG_LOG_FUNCTION_ID_BITMASK;
            result.functionObj = this.getFunctionObjById(result.functionId);
            result.text = "Enter: " + result.functionObj.name;
            break;
            
        case NRF_802154_LOG_TYPE_FUNCTION_EXIT:
            result.module = this.getModuleById((eventCode >> NRF_802154_DEBUG_LOG_MODULE_ID_BITPOS) & NRF_802154_DEBUG_LOG_MODULE_ID_BITMASK);            
            result.functionId = (eventCode >> NRF_802154_DEBUG_LOG_FUNCTION_ID_BITPOS) & NRF_802154_DEBUG_LOG_FUNCTION_ID_BITMASK;
            result.functionObj = this.getFunctionObjById(result.functionId);
            result.text = "Exit: " + result.functionObj.name;
            break;
            
        case NRF_802154_LOG_TYPE_LOCAL_EVENT:
            result.module = this.getModuleById((eventCode >> NRF_802154_DEBUG_LOG_MODULE_ID_BITPOS) & NRF_802154_DEBUG_LOG_MODULE_ID_BITMASK);
            result.localEvent = this.getLocalEventById(result.module, (eventCode >> NRF_802154_DEBUG_LOG_EVENT_ID_BITPOS) & NRF_802154_DEBUG_LOG_EVENT_ID_BITMASK);
            result.localEventParam = (eventCode >> NRF_802154_DEBUG_LOG_EVENT_PARAM_BITPOS) & NRF_802154_DEBUG_LOG_EVENT_PARAM_BITMASK;
            result.text = "Event: " + this.getEventText(result.localEvent, result.localEventParam);
            break;
            
        case NRF_802154_LOG_TYPE_GLOBAL_EVENT:
            result.module = this.getModuleById((eventCode >> NRF_802154_DEBUG_LOG_MODULE_ID_BITPOS) & NRF_802154_DEBUG_LOG_MODULE_ID_BITMASK);
            result.globalEvent = this.getGlobalEventById((eventCode >> NRF_802154_DEBUG_LOG_EVENT_ID_BITPOS) & NRF_802154_DEBUG_LOG_EVENT_ID_BITMASK);
            result.globalEventParam = (eventCode >> NRF_802154_DEBUG_LOG_EVENT_PARAM_BITPOS) & NRF_802154_DEBUG_LOG_EVENT_PARAM_BITMASK;
            result.text = "Event: " + this.getEventText(result.globalEvent, result.globalEventParam);
            break;
            
        default:
            result.module = this.createUnknownModule(0);
            result.text = "Unknown event type (" + eventCode + ")";
            break;
    }
    
    return result;
}
