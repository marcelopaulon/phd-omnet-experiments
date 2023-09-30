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

network DADCA_ACK_n50_u16_a8000m2_3
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
        @display("i=misc/sensor;p=852.33,7855.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2814.19,7911.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3417.62,7930.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4939.71,7866.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=612.05,7577.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1639.22,7537.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2013.75,7587.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2447.95,7545.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2960.03,7558.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3333.16,7543.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3815.31,7719.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4227.33,7661.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5604.35,7737.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1003.55,7494.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3958.96,7368.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4720.80,7310.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=112.13,7024.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2830.15,7179.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3225.76,7193.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4322.97,7165.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5145.96,7138.42");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=757.86,6927.43");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1149.10,6981.38");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1552.25,6982.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2444.89,6900.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3500.01,6800.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3880.57,6996.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4910.18,6838.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5459.34,6976.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5894.89,6814.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=320.61,6692.73");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1356.88,6665.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2001.56,6583.36");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3080.94,6580.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4590.93,6665.11");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5135.14,6567.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=97.54,6420.17");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=626.72,6464.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2415.70,6377.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4404.99,6349.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5874.63,6407.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=847.17,6078.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1206.80,6153.56");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1818.14,6255.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3293.24,6245.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3926.94,6204.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4963.82,6193.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5494.82,6232.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=35.95,5955.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=418.76,5856.88");
    }

}

