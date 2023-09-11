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

network DADCA_ACK_n50_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=174.64,81.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=120.70,204.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=45.26,250.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=50.08,347.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=134.12,346.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=210.00,332.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=235.07,409.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=154.14,409.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=286.04,479.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=366.35,459.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=434.63,415.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=223.75,557.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=157.51,532.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=94.20,456.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=199.11,659.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=103.16,662.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=63.77,615.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=4.26,629.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=202.31,789.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=265.71,828.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=265.86,766.76");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=140.14,614.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=270.13,656.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=199.07,458.70");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=78.44,542.37");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=314.36,726.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=224.09,721.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=177.39,861.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=100.30,858.04");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=32.77,822.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=85.67,771.97");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=440.98,744.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=471.67,800.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=438.53,854.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=399.74,927.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=485.31,897.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=547.64,871.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=518.62,953.74");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=609.97,970.95");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=567.80,1029.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=442.37,993.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=491.38,1156.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=447.98,1220.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=404.19,1287.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=338.35,1182.93");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=385.67,1139.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=275.00,1180.25");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=261.81,1103.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=195.82,1083.60");
    }

}

