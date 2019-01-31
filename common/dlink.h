#ifndef _DLINK_H_
#define _DLINK_H_

#include <stdint.h>

#define RFID_LEN        4

typedef struct _tagInfo
{
	uint8_t tagId[RFID_LEN];
	uint32_t aged;
} tagInfo;

typedef struct _tagNode
{
	tagInfo *msg;
	struct _tagNode *next;
} tagNode_t;

extern tagNode_t g_node_list;

void node_list_init(tagNode_t *node);
void node_list_add_head(tagNode_t *head, tagNode_t *node);
void node_list_add_tail(tagNode_t *head, tagNode_t *node);
void node_list_remove(tagNode_t *head, tagNode_t *node);
tagNode_t *node_list_find_tagid(tagNode_t *head, uint8_t *buf, uint8_t len);
tagNode_t *node_list_find_aged(tagNode_t *head, uint32_t current_ts);
void node_list_print(tagNode_t *head);

#endif
