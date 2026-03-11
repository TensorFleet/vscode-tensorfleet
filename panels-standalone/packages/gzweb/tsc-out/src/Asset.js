export var AssetError;
(function (AssetError) {
    AssetError["NOT_FOUND"] = "asset_not_found";
    AssetError["URI_MISSING"] = "asset_uri_missing";
})(AssetError || (AssetError = {}));
/**
 * Type that represents a simulation asset that needs to be fetched from a websocket server.
 */
export class Asset {
    constructor(uri, cb) {
        this.uri = uri;
        this.cb = cb;
    }
}
