import React, { useState, useEffect } from 'react';
import {
    Box,
    VStack,
    HStack,
    Button,
    Select,
    Input,
    Modal,
    ModalOverlay,
    ModalContent,
    ModalHeader,
    ModalFooter,
    ModalBody,
    ModalCloseButton,
    useDisclosure,
    useToast,
    Text,
    Badge,
    IconButton,
    Tooltip,
    Divider,
    FormControl,
    FormLabel,
    Textarea,
    SimpleGrid,
} from '@chakra-ui/react';
import ActionManager from '../action-manager/ActionManager';
import { layoutService, type LayoutConfig } from '../../services/layoutService';

export interface LayoutManagerProps {
    onLayoutChange?: (layout: LayoutConfig) => void;
}

const LayoutManager: React.FC<LayoutManagerProps> = ({ onLayoutChange }) => {
    const [currentLayout, setCurrentLayout] = useState<LayoutConfig | null>(null);
    const [availableLayouts, setAvailableLayouts] = useState<LayoutConfig[]>([]);
    const [newLayoutName, setNewLayoutName] = useState<string>('');
    const [newLayoutDescription, setNewLayoutDescription] = useState<string>('');

    const { isOpen: isNewLayoutOpen, onOpen: onNewLayoutOpen, onClose: onNewLayoutClose } = useDisclosure();

    const toast = useToast();

    // Load layouts on component mount
    useEffect(() => {
        loadLayouts();
    }, []);

    const loadLayouts = () => {
        try {
            const layouts = layoutService.getAllLayouts();
            setAvailableLayouts(layouts);

            // Load active layout or create default
            const activeLayout = layoutService.getOrCreateDefaultLayout();
            setCurrentLayout(activeLayout);

            if (onLayoutChange) {
                onLayoutChange(activeLayout);
            }
        } catch (error) {
            console.error('Error loading layouts:', error);
            toast({
                title: 'Error loading layouts',
                description: 'Failed to load saved layouts',
                status: 'error',
                duration: 3000,
                isClosable: true,
            });
        }
    };

    const handleLayoutSelect = (layoutId: string) => {
        try {
            const layout = layoutService.getLayout(layoutId);
            if (layout) {
                setCurrentLayout(layout);
                layoutService.setActiveLayout(layoutId);

                if (onLayoutChange) {
                    onLayoutChange(layout);
                }

                toast({
                    title: 'Layout loaded',
                    description: `Switched to "${layout.name}"`,
                    status: 'success',
                    duration: 2000,
                    isClosable: true,
                });
            }
        } catch (error) {
            console.error('Error loading layout:', error);
            toast({
                title: 'Error loading layout',
                description: 'Failed to load the selected layout',
                status: 'error',
                duration: 3000,
                isClosable: true,
            });
        }
    };

    const handleCreateLayout = () => {
        if (!newLayoutName.trim()) {
            toast({
                title: 'Name required',
                description: 'Please enter a name for the layout',
                status: 'warning',
                duration: 3000,
                isClosable: true,
            });
            return;
        }

        try {
            const newLayout = layoutService.createLayout(newLayoutName.trim(), newLayoutDescription.trim());
            setCurrentLayout(newLayout);
            layoutService.setActiveLayout(newLayout.id);
            loadLayouts();

            setNewLayoutName('');
            setNewLayoutDescription('');
            onNewLayoutClose();

            if (onLayoutChange) {
                onLayoutChange(newLayout);
            }

            toast({
                title: 'Layout created',
                description: `Created new layout "${newLayout.name}"`,
                status: 'success',
                duration: 3000,
                isClosable: true,
            });
        } catch (error) {
            console.error('Error creating layout:', error);
            toast({
                title: 'Error creating layout',
                description: 'Failed to create the new layout',
                status: 'error',
                duration: 3000,
                isClosable: true,
            });
        }
    };


    const handleDeleteLayout = (layoutId: string) => {
        if (availableLayouts.length <= 1) {
            toast({
                title: 'Cannot delete',
                description: 'Cannot delete the last remaining layout',
                status: 'warning',
                duration: 3000,
                isClosable: true,
            });
            return;
        }

        try {
            layoutService.deleteLayout(layoutId);
            loadLayouts();

            // If we deleted the current layout, switch to the first available one
            if (currentLayout?.id === layoutId) {
                const remainingLayouts = layoutService.getAllLayouts();
                if (remainingLayouts.length > 0) {
                    const firstLayout = remainingLayouts[0];
                    setCurrentLayout(firstLayout);
                    layoutService.setActiveLayout(firstLayout.id);

                    if (onLayoutChange) {
                        onLayoutChange(firstLayout);
                    }
                }
            }

            toast({
                title: 'Layout deleted',
                description: 'Layout has been removed',
                status: 'success',
                duration: 2000,
                isClosable: true,
            });
        } catch (error) {
            console.error('Error deleting layout:', error);
            toast({
                title: 'Error deleting layout',
                description: 'Failed to delete the layout',
                status: 'error',
                duration: 3000,
                isClosable: true,
            });
        }
    };

    const handleAddPanel = () => {
        if (!currentLayout) return;

        try {
            const panelCount = currentLayout.panelOrder?.length || 0;
            const newPanel = layoutService.addPanel(currentLayout.id, {
                title: `Action Manager ${panelCount + 1}`,
                collapsed: false
            });

            if (newPanel) {
                // Reload the layout to get the updated state
                const updatedLayout = layoutService.getLayout(currentLayout.id);
                if (updatedLayout) {
                    setCurrentLayout(updatedLayout);
                    if (onLayoutChange) {
                        onLayoutChange(updatedLayout);
                    }
                }

                toast({
                    title: 'Panel added',
                    description: 'New action manager panel added to layout',
                    status: 'success',
                    duration: 2000,
                    isClosable: true,
                });
            }
        } catch (error) {
            console.error('Error adding panel:', error);
            toast({
                title: 'Error adding panel',
                description: 'Failed to add new panel',
                status: 'error',
                duration: 3000,
                isClosable: true,
            });
        }
    };

    const handleRemovePanel = (panelId: string) => {
        if (!currentLayout) return;

        try {
            const success = layoutService.removePanel(currentLayout.id, panelId);
            if (success) {
                // Reload the layout to get the updated state
                const updatedLayout = layoutService.getLayout(currentLayout.id);
                if (updatedLayout) {
                    setCurrentLayout(updatedLayout);
                    if (onLayoutChange) {
                        onLayoutChange(updatedLayout);
                    }
                }

                toast({
                    title: 'Panel removed',
                    description: 'Action manager panel removed from layout',
                    status: 'success',
                    duration: 2000,
                    isClosable: true,
                });
            }
        } catch (error) {
            console.error('Error removing panel:', error);
            toast({
                title: 'Error removing panel',
                description: 'Failed to remove panel',
                status: 'error',
                duration: 3000,
                isClosable: true,
            });
        }
    };


    const handleColumnsChange = (columns: number) => {
        if (!currentLayout) return;

        try {
            const success = layoutService.updateLayoutConfig(currentLayout.id, {
                columnsPerRow: columns
            });

            if (success) {
                const updatedLayout = layoutService.getLayout(currentLayout.id);
                if (updatedLayout) {
                    setCurrentLayout(updatedLayout);
                    if (onLayoutChange) {
                        onLayoutChange(updatedLayout);
                    }
                }
            }
        } catch (error) {
            console.error('Error updating layout config:', error);
        }
    };

    if (!currentLayout) {
        return (
            <Box p={4}>
                <Text>Loading layout...</Text>
            </Box>
        );
    }

    return (
        <Box w="100%" minH="100vh">
            {/* Layout Controls */}
            <VStack spacing={4} align="stretch" mb={4} px={4} pt={4}>
                <HStack justify="space-between" align="center">
                    <HStack spacing={4}>
                        <Text fontSize="lg" fontWeight="bold">
                            Layout Manager
                        </Text>
                        <Badge colorScheme="blue" variant="subtle">
                            {currentLayout.panelOrder?.length || 0} panels
                        </Badge>
                    </HStack>

                    <HStack spacing={2}>
                        <Button size="sm" colorScheme="green" onClick={handleAddPanel}>
                            ➕ Add Panel
                        </Button>
                        <Button size="sm" colorScheme="blue" onClick={onNewLayoutOpen}>
                            📄 New Layout
                        </Button>
                    </HStack>
                </HStack>

                <HStack spacing={4} align="center">
                    <FormControl maxW="300px">
                        <FormLabel fontSize="sm">Current Layout:</FormLabel>
                        <Select
                            value={currentLayout.id}
                            onChange={(e) => handleLayoutSelect(e.target.value)}
                            size="sm"
                        >
                            {availableLayouts.map((layout) => (
                                <option key={layout.id} value={layout.id}>
                                    {layout.name}
                                </option>
                            ))}
                        </Select>
                    </FormControl>

                    <Text fontSize="sm" color="gray.600">
                        {currentLayout.description}
                    </Text>

                    {availableLayouts.length > 1 && (
                        <Tooltip label="Delete current layout">
                            <IconButton
                                aria-label="Delete layout"
                                icon={<Text>🗑️</Text>}
                                size="sm"
                                variant="ghost"
                                colorScheme="red"
                                onClick={() => handleDeleteLayout(currentLayout.id)}
                            />
                        </Tooltip>
                    )}
                </HStack>

            </VStack>

            <Divider mb={4} />

            {/* Flow-based Layout */}
            <Box
                w="100%"
                px={4}
                pb={4}
            >
                <SimpleGrid
                    spacing={4}
                    templateColumns="repeat(auto-fit, minmax(300px, 1fr))"
                    w="100%"
                    maxW="100%"
                >
                    {(currentLayout.panelOrder || []).map((panelId) => {
                        const panel = currentLayout.panels[panelId];

                        if (!panel) {
                            console.warn(`Panel ${panelId} not found in panels object`);
                            return null;
                        }

                        return (
                            <Box
                                key={panelId}
                                position="relative"
                                border="1px solid"
                                borderColor="gray.200"
                                borderRadius="md"
                                overflow="hidden"
                                minH="300px"
                                bg="white"
                                sx={{
                                    _dark: {
                                        borderColor: "gray.600",
                                        bg: "gray.800"
                                    }
                                }}
                            >
                                <ActionManager
                                    id={panel.id}
                                    layoutId={currentLayout.id}
                                    title={panel.title}
                                    defaultNamespace={panel.defaultNamespace}
                                    allowedActions={panel.allowedActions}
                                />

                                {/* Remove Panel Button */}
                                <IconButton
                                    aria-label="Remove panel"
                                    icon={<Text fontSize="xs">❌</Text>}
                                    size="xs"
                                    position="absolute"
                                    top={2}
                                    right={2}
                                    variant="ghost"
                                    colorScheme="red"
                                    onClick={() => handleRemovePanel(panel.id)}
                                    zIndex={10}
                                    bg="white"
                                    sx={{
                                        _dark: { bg: "gray.800" },
                                        _hover: {
                                            bg: "red.50",
                                            _dark: { bg: "red.900" }
                                        }
                                    }}
                                />
                            </Box>
                        );
                    })}
                </SimpleGrid>

                {/* Show message if no panels */}
                {(!currentLayout.panelOrder || currentLayout.panelOrder.length === 0) && (
                    <Box textAlign="center" py={8}>
                        <Text color="gray.500">
                            No panels in this layout. Click "Add Panel" to get started.
                        </Text>
                    </Box>
                )}
            </Box>

            {/* New Layout Modal */}
            <Modal isOpen={isNewLayoutOpen} onClose={onNewLayoutClose}>
                <ModalOverlay />
                <ModalContent>
                    <ModalHeader>Create New Layout</ModalHeader>
                    <ModalCloseButton />
                    <ModalBody>
                        <VStack spacing={4}>
                            <FormControl>
                                <FormLabel>Layout Name</FormLabel>
                                <Input
                                    value={newLayoutName}
                                    onChange={(e) => setNewLayoutName(e.target.value)}
                                    placeholder="Enter layout name"
                                />
                            </FormControl>
                            <FormControl>
                                <FormLabel>Description (optional)</FormLabel>
                                <Textarea
                                    value={newLayoutDescription}
                                    onChange={(e) => setNewLayoutDescription(e.target.value)}
                                    placeholder="Enter layout description"
                                    rows={3}
                                />
                            </FormControl>
                        </VStack>
                    </ModalBody>
                    <ModalFooter>
                        <Button variant="ghost" mr={3} onClick={onNewLayoutClose}>
                            Cancel
                        </Button>
                        <Button colorScheme="blue" onClick={handleCreateLayout}>
                            Create Layout
                        </Button>
                    </ModalFooter>
                </ModalContent>
            </Modal>

        </Box>
    );
};

export default LayoutManager;
