<?php if (!defined('APPLICATION')) exit();

// Conversations
$Configuration['Conversations']['Version'] = '2.0.17.10';

// Database
$Configuration['Database']['Name'] = 'sparta_van285';
$Configuration['Database']['Host'] = 'localhost';
$Configuration['Database']['User'] = 'sparta_van285';
$Configuration['Database']['Password'] = 'ac18clSb5P';

// EnabledApplications
$Configuration['EnabledApplications']['Conversations'] = 'conversations';
$Configuration['EnabledApplications']['Vanilla'] = 'vanilla';

// EnabledPlugins
$Configuration['EnabledPlugins']['GettingStarted'] = 'GettingStarted';
$Configuration['EnabledPlugins']['HtmLawed'] = 'HtmLawed';
$Configuration['EnabledPlugins']['Flagging'] = 'Flagging';
$Configuration['EnabledPlugins']['Minify'] = 'Minify';
$Configuration['EnabledPlugins']['embedvanilla'] = 'embedvanilla';

// Garden
$Configuration['Garden']['Title'] = 'Spartabots';
$Configuration['Garden']['Cookie']['Salt'] = 'i9uslyqw';
$Configuration['Garden']['Cookie']['Domain'] = '';
$Configuration['Garden']['Version'] = '2.0.17.10';
$Configuration['Garden']['RewriteUrls'] = TRUE;
$Configuration['Garden']['CanProcessImages'] = TRUE;
$Configuration['Garden']['Installed'] = TRUE;
$Configuration['Garden']['Errors']['MasterView'] = 'error.master.php';
$Configuration['Garden']['InstallationID'] = 'F91B-F98DBBF0-14DD9899';
$Configuration['Garden']['InstallationSecret'] = 'faf0807d977ff0caf25dc16039df8958985dd681';
$Configuration['Garden']['Analytics']['LastSentDate'] = '20110815';
$Configuration['Garden']['Registration']['Method'] = 'Captcha';
$Configuration['Garden']['Registration']['CaptchaPrivateKey'] = '6LdyHMcSAAAAAG7EjjCA2zADyAlnxPNuSWsEcM2N ';
$Configuration['Garden']['Registration']['CaptchaPublicKey'] = '6LdyHMcSAAAAAGqZ1jxQrRNSf8-K_D16VMwIxXW2 ';
$Configuration['Garden']['Registration']['InviteExpiration'] = '-1 week';
$Configuration['Garden']['Registration']['InviteRoles'] = 'a:3:{i:8;s:1:"0";i:32;s:1:"0";i:16;s:1:"0";}';
$Configuration['Garden']['Theme'] = 'EmbedFriendly';

// Modules
$Configuration['Modules']['Vanilla']['Content'] = 'a:6:{i:0;s:13:"MessageModule";i:1;s:7:"Notices";i:2;s:21:"NewConversationModule";i:3;s:19:"NewDiscussionModule";i:4;s:7:"Content";i:5;s:3:"Ads";}';
$Configuration['Modules']['Conversations']['Content'] = 'a:6:{i:0;s:13:"MessageModule";i:1;s:7:"Notices";i:2;s:21:"NewConversationModule";i:3;s:19:"NewDiscussionModule";i:4;s:7:"Content";i:5;s:3:"Ads";}';

// Plugins
$Configuration['Plugins']['GettingStarted']['Dashboard'] = '1';
$Configuration['Plugins']['GettingStarted']['Registration'] = '1';
$Configuration['Plugins']['GettingStarted']['Categories'] = '1';
$Configuration['Plugins']['GettingStarted']['Plugins'] = '1';
$Configuration['Plugins']['GettingStarted']['Discussion'] = '1';
$Configuration['Plugins']['EmbedVanilla']['RemoteUrl'] = 'http://spartabots.co.cc/forums.html';

// Routes
$Configuration['Routes']['DefaultController'] = 'discussions';

// Vanilla
$Configuration['Vanilla']['Version'] = '2.0.17.10';

// Last edited by Unknown (67.182.140.4)2011-08-16 10:22:00