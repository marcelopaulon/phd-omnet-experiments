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

network DADCA_ACK_n50_u8_a8000m2_3
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
        @display("i=misc/sensor;p=207.04,7954.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2039.67,7831.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3016.84,7856.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3803.66,7945.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4155.81,7930.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4619.65,7815.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4993.56,7990.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5768.47,7858.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=114.52,7571.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=574.75,7733.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1006.94,7600.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1684.41,7745.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5197.90,7672.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1178.15,7262.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2180.82,7499.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2682.18,7504.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3522.11,7466.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4240.56,7503.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4679.14,7451.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5303.31,7317.43");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5823.07,7431.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=403.11,7168.34");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1861.02,7175.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3128.19,7018.17");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3590.21,7023.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3917.81,7191.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=53.40,6846.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=891.79,6791.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2444.63,6831.94");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4196.44,6858.24");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4802.05,6803.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5929.65,7004.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=312.85,6542.06");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1423.24,6633.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1861.46,6754.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3723.26,6629.38");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5240.30,6564.44");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2132.23,6449.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2840.70,6483.36");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3405.84,6422.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5575.47,6442.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5947.55,6364.07");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1122.93,6217.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3201.36,6104.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3680.61,6050.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4118.33,6135.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5015.34,6190.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=613.88,6013.25");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1432.04,5979.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1783.82,5901.75");
    }

}

