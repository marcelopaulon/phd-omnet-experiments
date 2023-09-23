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

network DADCA_ACK_n50_u4_a8000m2_2
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
        @display("i=misc/sensor;p=58.68,7887.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=579.11,7871.57");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1816.52,7860.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2505.10,7915.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3556.06,7768.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4220.94,7933.34");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=918.85,7708.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1302.08,7619.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2255.31,7659.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4713.67,7666.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=153.71,7334.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=642.96,7461.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1659.88,7465.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2809.07,7397.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3721.54,7397.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4219.49,7460.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4971.21,7304.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5568.76,7474.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1241.57,7190.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2511.68,7112.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2922.33,7051.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3277.03,7120.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4143.83,7089.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4551.79,7123.81");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5462.44,7049.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5905.28,7118.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=212.65,6943.23");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=564.15,6811.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1845.29,6886.61");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3686.42,6878.27");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5086.53,6786.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2296.47,6545.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4264.07,6733.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4636.96,6706.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5460.49,6614.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5899.66,6664.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=122.70,6465.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=579.55,6404.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=952.59,6460.71");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1547.37,6422.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1928.54,6339.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2840.37,6472.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3828.89,6271.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4174.60,6393.38");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4733.35,6295.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1053.12,6046.12");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2197.85,6046.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2546.52,6219.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3059.34,6099.23");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3476.23,6231.14");
    }

}

