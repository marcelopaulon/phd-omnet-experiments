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

network DADCA_ACK_n50_u16_a8000m2_5
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
        @display("i=misc/sensor;p=1372.87,7774.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2207.32,7828.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2584.24,7875.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4026.24,7935.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4444.47,7831.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4880.69,7767.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=737.62,7578.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1819.56,7621.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3211.50,7614.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3596.23,7686.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1067.22,7285.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2466.40,7367.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3954.76,7280.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4295.95,7407.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4974.19,7276.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5835.14,7370.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=690.06,7208.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2056.91,7252.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2952.36,7019.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3467.23,7034.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4645.23,7111.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=51.54,7001.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=486.17,6865.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1680.60,6968.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2131.34,6854.76");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2592.92,6948.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4015.56,6849.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5339.20,6971.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1094.87,6657.76");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2468.69,6530.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2825.01,6556.92");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3488.93,6515.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4293.14,6599.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4827.73,6676.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=57.55,6439.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=615.08,6311.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1754.79,6449.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4673.96,6307.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5123.01,6395.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5663.26,6434.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=282.70,6138.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2566.25,6126.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3191.13,6160.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3796.52,6193.35");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4357.12,6029.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4954.50,6077.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5323.76,6080.46");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5825.04,6098.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=575.92,5834.83");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1590.19,5964.17");
    }

}

