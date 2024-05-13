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

network DADCA_ACK_n50_u4_a8000m2_8
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
        @display("i=misc/sensor;p=763.68,7967.55");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2005.82,7953.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2763.13,7982.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3646.05,7890.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4066.12,7920.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5276.69,7919.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=158.54,7548.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=738.42,7610.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2397.78,7683.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2830.20,7567.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5128.18,7546.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5785.34,7654.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=539.90,7308.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1159.33,7424.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1838.58,7425.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2431.50,7303.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3916.47,7473.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4484.35,7395.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=270.34,7030.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1041.36,7020.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3177.63,7130.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4830.42,7126.07");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5419.69,7248.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5873.89,7227.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1394.82,7006.88");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2016.62,6942.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2641.19,7002.26");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3590.76,6934.19");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4094.82,6841.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5315.95,6836.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=95.19,6691.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=509.29,6656.23");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1753.35,6640.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2331.37,6751.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2917.88,6539.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4493.44,6635.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5707.11,6731.19");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=209.41,6283.24");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=800.63,6386.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1383.44,6326.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2411.92,6394.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3400.25,6507.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3885.28,6465.00");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5012.85,6467.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1870.67,6167.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3071.49,6182.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4164.09,6053.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5806.33,6259.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=838.64,5795.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1309.00,5915.42");
    }

}

