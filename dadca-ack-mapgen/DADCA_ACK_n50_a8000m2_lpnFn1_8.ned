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

network DADCA_ACK_n50_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=55.18,146.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=91.85,238.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=178.70,214.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=49.70,33.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=103.28,3.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=172.88,147.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=171.02,29.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=243.42,31.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=296.06,66.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=353.46,45.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=394.75,113.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=373.99,175.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=8.85,346.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=47.62,292.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=58.75,472.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=91.94,537.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=173.83,516.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=234.22,480.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=222.29,611.96");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=134.19,618.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=237.85,693.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=167.23,684.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=83.45,668.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=99.93,780.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=22.17,708.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=2.81,785.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=281.01,584.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=335.78,621.79");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=418.86,591.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=22.91,561.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=189.88,424.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=131.03,472.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=90.39,372.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=216.92,362.59");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=277.31,393.47");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=316.79,501.99");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=370.28,550.89");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=307.36,768.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=237.31,820.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=299.61,879.96");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=211.08,949.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=280.99,1017.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=323.79,1083.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=414.06,1091.80");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=490.10,1139.35");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=519.92,1066.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=591.66,1061.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=641.68,1011.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=666.87,927.93");
    }

}

