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

network DADCA_ACK_n50_u8_a8000m2_6
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
        @display("i=misc/sensor;p=271.14,7762.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1158.14,7911.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1991.25,7914.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2676.53,7964.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4539.43,7811.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5236.49,7930.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5858.48,7892.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=600.81,7609.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1696.10,7529.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2484.10,7548.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3127.06,7730.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3513.06,7678.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4137.42,7548.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5979.93,7533.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1092.70,7311.59");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2198.74,7319.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3003.34,7381.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4780.96,7414.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5661.71,7382.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=221.80,7022.59");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=678.89,7158.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1470.49,7242.76");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3366.52,7257.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5956.69,7064.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1236.65,6778.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1771.13,6940.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2382.72,7007.76");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2810.36,7002.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4111.37,6964.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4888.47,6889.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5353.64,6922.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=209.20,6672.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=677.57,6747.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2783.19,6615.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3251.10,6742.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3781.93,6735.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5791.70,6570.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=211.57,6303.59");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1231.51,6397.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1607.50,6414.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2251.27,6503.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3253.70,6276.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4228.51,6285.94");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5044.75,6469.50");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5413.82,6463.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=642.37,6205.37");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4702.83,6169.65");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5376.23,6074.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=240.80,5811.88");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=972.67,5894.87");
    }

}

