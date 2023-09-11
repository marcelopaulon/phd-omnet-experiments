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

network DADCA_ACK_n50_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=53.41,158.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=113.49,182.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=159.20,240.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=133.72,46.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=41.04,85.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=25.91,10.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=214.09,65.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=224.94,135.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=225.69,279.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=200.02,347.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=329.62,356.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=390.07,363.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=448.69,443.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=476.75,507.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=538.96,531.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=551.84,459.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=615.92,416.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=672.41,479.78");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=732.88,542.42");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=690.88,588.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=614.82,589.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=501.96,617.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=483.46,709.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=552.59,722.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=436.38,639.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=363.57,613.33");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=276.37,564.43");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=243.21,624.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=340.53,524.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=250.83,494.44");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=176.55,538.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=110.33,534.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=68.25,450.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=66.49,373.47");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=145.11,417.49");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=229.26,404.95");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=244.96,12.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=294.18,71.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=352.81,169.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=321.49,244.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=440.59,247.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=487.65,355.59");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=343.32,460.97");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=283.80,692.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=279.97,787.69");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=205.52,721.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=351.28,722.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=455.77,564.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=401.63,535.79");
    }

}

