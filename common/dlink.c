#include "dlink.h"
#include "stdlib.h"
#include "string.h"
#include "FreeRTOS.h"
#include "dbg_print.h"

void node_list_init(tagNode_t *node)
{
	node->msg = NULL;
	node->next = NULL;
}

void node_list_add_head(tagNode_t *head, tagNode_t *node)
{
	node->next = head->next;
	head->next = node;
}

void node_list_add_tail(tagNode_t *head, tagNode_t *node)
{
	tagNode_t *curNode = head;
	for (curNode = head; curNode->next != NULL; curNode = curNode->next);
	curNode->next = node;
	node->next = NULL;
}

void node_list_remove(tagNode_t *head, tagNode_t *node)
{
	tagNode_t *prvNode, *curNode;
	prvNode = head;
	for (curNode = head->next; curNode != NULL; curNode = curNode->next)
	{
		if (curNode == node)
		{
			prvNode->next = curNode->next;
			vPortFree(curNode->msg);
			vPortFree(curNode);
			break;
		}
		prvNode = curNode;
	}
}

tagNode_t *node_list_find_tagid(tagNode_t *head, uint8_t *buf, uint8_t len)
{
	tagNode_t *curNode;
	for (curNode = head->next; curNode != NULL; curNode = curNode->next)
	{
		if (memcmp(curNode->msg->tagId, buf, len) == 0)
			return curNode;
	}
	return NULL;
}

tagNode_t *node_list_find_aged(tagNode_t *head, uint32_t current_ts)
{
	tagNode_t *curNode;
	for (curNode = head->next; curNode != NULL; curNode = curNode->next)
	{
		if (current_ts - curNode->msg->aged > 60000)
			return curNode;
	}
	return NULL;
}

void node_list_print(tagNode_t *head)
{
	uint8_t i;
	tagNode_t *curNode;
	dbg_print(PRINT_LEVEL_DEBUG, "print a node list:\r\n");
	for (curNode = head->next; curNode != NULL; curNode = curNode->next)
	{
		dbg_print(PRINT_LEVEL_DEBUG, "tag id: ");
		for (i = 0; i < 4; i++)
			dbg_print(PRINT_LEVEL_DEBUG, "%02X ", curNode->msg->tagId[i]);

		dbg_print(PRINT_LEVEL_DEBUG, "\ttag aded: %d\r\n", curNode->msg->aged);
	}
}
