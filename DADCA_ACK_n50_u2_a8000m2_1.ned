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

network DADCA_ACK_n50_u2_a8000m2_1
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
        @display("i=misc/sensor;p=861.42,7868.56");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1261.96,7867.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1730.84,7802.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2163.40,7958.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2527.82,7932.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3180.38,7832.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3763.18,7940.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4407.38,7862.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4965.24,7914.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5608.11,7778.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=347.68,7539.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1035.48,7527.79");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2807.74,7710.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3696.03,7528.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4088.06,7553.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4678.93,7585.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1400.74,7270.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1760.26,7269.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2255.39,7495.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3348.14,7404.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5144.90,7267.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=812.93,7231.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4592.05,7209.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1235.46,6860.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1928.81,6840.49");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2760.28,6846.63");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3111.02,6999.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3527.26,6835.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4009.88,6980.44");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4464.25,6830.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5513.72,6957.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=496.52,6669.42");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=880.08,6701.99");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2429.96,6649.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3850.81,6652.01");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4964.61,6605.64");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5708.03,6656.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=702.67,6380.16");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1370.63,6396.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1973.75,6427.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3246.25,6371.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5252.74,6344.97");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5636.15,6305.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1754.76,6054.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2808.43,6156.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4109.88,6231.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4762.98,6027.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=338.98,5841.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=853.76,6017.01");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2261.16,6007.19");
    }

}

