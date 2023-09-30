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

network DADCA_ACK_n50_u16_a8000m2_8
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
        @display("i=misc/sensor;p=1400.61,7911.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3760.28,7979.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4357.39,7941.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=102.16,7673.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1026.36,7711.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3395.85,7610.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3795.09,7580.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4684.79,7615.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5528.78,7655.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=491.80,7301.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1400.57,7463.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4210.51,7265.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5137.41,7480.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=23.43,7212.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1044.33,7077.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1708.42,7140.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2933.25,7033.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3502.18,7043.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5321.80,7078.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5722.58,7129.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2096.65,6903.74");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2550.28,6812.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4325.51,6841.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4793.02,6970.76");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=29.25,6736.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=763.09,6702.91");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1393.94,6628.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2932.71,6585.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3293.70,6613.85");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3673.15,6718.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5723.64,6718.27");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=370.49,6508.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1035.46,6459.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1715.06,6334.69");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2320.88,6419.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3907.11,6339.89");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5245.32,6499.68");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=710.71,6154.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1150.67,6039.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2263.59,6041.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3011.21,6094.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3342.87,6246.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4163.44,6095.00");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5117.57,6096.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5696.29,6029.95");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3572.00,5884.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=875.63,5562.26");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1414.57,5666.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1949.02,5591.42");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2729.31,5587.39");
    }

}

