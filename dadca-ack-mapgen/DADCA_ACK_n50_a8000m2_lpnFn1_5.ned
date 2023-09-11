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

network DADCA_ACK_n50_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=190.13,133.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=231.64,196.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=300.62,292.77");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=402.79,255.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=443.31,307.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=515.95,346.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=592.44,334.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=567.86,418.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=519.82,489.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=449.18,447.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=394.79,484.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=329.09,538.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=347.72,442.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=251.64,451.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=255.47,554.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=216.40,627.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=170.39,580.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=110.20,598.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=47.49,643.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=12.75,585.11");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=99.52,496.43");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=135.13,444.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=73.23,414.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=13.31,418.49");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=13.82,349.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=89.49,289.03");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=107.82,195.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=316.59,147.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=185.40,311.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=160.80,376.17");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=75.24,350.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=277.69,687.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=301.39,606.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=210.28,738.78");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=305.30,756.31");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=351.09,834.08");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=283.98,863.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=237.44,938.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=252.28,1016.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=330.26,1036.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=376.37,1084.67");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=438.44,1157.66");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=524.79,1114.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=576.80,1149.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=633.39,1075.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=499.98,986.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=434.91,1017.27");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=360.02,1157.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=287.64,1208.61");
    }

}

