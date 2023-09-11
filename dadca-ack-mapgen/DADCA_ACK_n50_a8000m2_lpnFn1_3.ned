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

network DADCA_ACK_n50_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=70.15,174.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=74.40,261.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=136.67,329.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=179.72,264.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=246.88,387.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=290.76,456.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=273.30,315.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=338.42,358.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=369.59,436.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=389.72,244.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=401.30,325.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=442.84,383.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=448.72,467.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=355.86,501.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=404.61,544.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=447.94,643.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=386.47,670.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=324.61,601.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=271.85,655.73");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=226.47,458.82");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=152.36,432.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=285.09,536.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=217.03,578.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=324.70,693.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=265.40,767.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=202.75,839.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=110.75,830.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=74.19,921.10");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=135.04,891.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=53.18,803.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=0.81,838.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=122.87,708.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=172.87,780.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=29.61,707.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=45.86,621.27");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=216.64,695.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=373.88,800.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=431.68,820.71");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=464.26,891.48");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=525.91,932.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=536.55,1014.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=474.76,1039.77");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=385.31,1024.59");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=317.25,1055.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=349.49,1132.67");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=434.34,989.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=495.43,1129.01");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=437.20,1206.41");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=456.14,1278.97");
    }

}

