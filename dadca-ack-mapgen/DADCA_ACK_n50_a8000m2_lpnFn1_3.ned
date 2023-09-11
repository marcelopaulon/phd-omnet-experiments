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
        @display("i=misc/sensor,p=43.44,132.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=102.67,183.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=178.54,195.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=33.59,198.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=1.99,262.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=5.91,337.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=98.97,366.07");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=139.60,440.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=231.68,441.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=182.27,489.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=136.01,562.83");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=89.64,497.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=6.22,412.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=84.78,283.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=156.51,72.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=261.26,255.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=351.98,222.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=433.61,209.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=489.91,270.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=464.86,117.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=424.60,274.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=427.53,358.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=470.73,408.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=543.18,348.49");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=301.40,317.72");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=368.45,334.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=244.99,359.49");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=253.01,574.74");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=320.48,588.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=323.37,517.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=402.01,506.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=368.03,431.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=542.57,463.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=603.66,437.11");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=626.29,370.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=690.02,404.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=714.99,490.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=645.70,557.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=720.61,558.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=567.70,616.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=643.16,627.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=546.27,535.48");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=478.41,519.34");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=495.80,587.60");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=464.83,676.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=398.91,697.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=650.02,297.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=726.05,355.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=723.26,291.99");
    }

}

