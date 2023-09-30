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

network DADCA_ACK_n50_u2_a8000m2_5
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
        @display("i=misc/sensor;p=468.13,7772.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1313.02,7778.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1724.65,7904.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2073.85,7978.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2750.71,7893.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3492.06,7823.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4504.87,7821.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2348.40,7687.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3839.94,7613.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4989.14,7692.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=78.33,7296.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=437.88,7315.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1077.76,7482.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1679.04,7472.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2191.73,7314.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2580.43,7310.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2947.33,7461.05");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3572.49,7374.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4153.90,7433.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4691.89,7449.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5196.64,7289.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5583.66,7452.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=764.05,7145.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1135.44,7092.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1804.48,7105.26");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4631.67,7063.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=286.12,6923.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1462.74,6875.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3403.67,6959.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3822.40,6986.81");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5142.37,6908.10");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5560.28,6908.58");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=620.24,6699.52");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2114.35,6696.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2850.02,6727.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3240.49,6625.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4774.24,6685.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5738.81,6544.59");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=329.91,6307.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1128.94,6323.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1758.95,6454.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2563.07,6383.73");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3575.34,6470.79");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4051.81,6456.80");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4434.16,6354.35");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5087.94,6435.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5506.37,6280.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=887.32,6021.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2919.12,6038.53");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3414.95,6145.65");
    }

}

