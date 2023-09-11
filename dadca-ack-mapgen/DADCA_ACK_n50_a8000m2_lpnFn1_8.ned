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
        @display("i=misc/sensor,p=49.27,34.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=1.65,111.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=186.48,85.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=238.89,32.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=177.07,22.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=138.83,151.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=209.09,168.55");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=79.10,173.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=129.19,265.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=319.39,126.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=413.16,127.39");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=372.69,82.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=458.55,46.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=305.00,26.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=276.98,220.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=340.40,239.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=331.39,309.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=321.69,375.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=278.35,428.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=172.63,459.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=198.48,375.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=257.81,366.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=469.36,196.77");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=534.59,177.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=510.03,105.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=397.04,24.12");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=244.81,295.89");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=193.31,243.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=68.60,257.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=2.84,295.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=2.02,227.91");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=52.71,365.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=405.82,331.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=396.99,270.16");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=476.34,313.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=557.25,328.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=626.64,276.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=700.29,211.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=761.28,167.60");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=782.26,253.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=828.90,96.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=817.70,22.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=896.22,35.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=901.66,209.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=995.43,215.65");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1059.77,236.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1145.42,275.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1224.04,217.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1158.10,210.03");
    }

}

