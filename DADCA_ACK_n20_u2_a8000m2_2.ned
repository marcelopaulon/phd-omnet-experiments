package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_u2_a8000m2_2
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=61.07,7860.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=506.88,7979.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1046.87,7851.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2429.26,7871.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2816.12,7904.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3631.37,7960.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4164.82,7971.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4579.35,7776.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5108.89,7838.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5800.11,7791.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=431.87,7571.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1365.65,7577.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1889.87,7724.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2603.94,7531.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3092.59,7657.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3662.63,7606.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4204.61,7618.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5496.25,7558.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=692.48,7295.29");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2204.60,7464.34");
    }

}

