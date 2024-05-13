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

network DADCA_ACK_n50_u2_a8000m2_2
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
        @display("i=misc/sensor;p=158.91,7954.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=567.10,7965.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1097.84,7822.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1579.52,7941.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1904.52,7801.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2477.32,7941.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2797.81,7764.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3367.91,7762.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3868.61,7801.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4815.90,7878.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5271.21,7762.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=377.65,7608.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=754.43,7590.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1509.72,7591.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4467.63,7712.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5681.03,7700.56");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1077.76,7443.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1792.69,7352.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2508.86,7438.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3942.12,7386.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4385.38,7304.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4933.02,7312.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5481.31,7319.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=168.81,7227.65");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=710.80,7242.82");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1125.01,7059.07");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2425.99,7093.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3039.37,7207.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5265.84,7032.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5901.27,7135.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=445.92,6898.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1509.31,6922.44");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3616.40,6832.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4155.86,6823.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4503.85,6958.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4918.43,6812.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5572.22,6794.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=255.75,6573.64");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=606.89,6546.79");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1849.24,6699.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2620.77,6753.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2933.79,6519.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4482.13,6582.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5321.12,6515.30");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1233.37,6457.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2211.63,6461.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4078.78,6409.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=6.46,6261.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=726.15,6185.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2610.67,6116.49");
    }

}

