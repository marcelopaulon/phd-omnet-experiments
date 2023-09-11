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
        @display("i=misc/sensor,p=24.45,104.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=143.08,160.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=193.38,232.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=256.21,216.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=333.40,266.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=376.78,349.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=286.73,309.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=268.52,449.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=348.40,470.39");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=408.84,515.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=325.44,570.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=261.29,532.75");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=183.29,543.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=339.39,695.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=407.14,763.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=392.46,650.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=471.06,591.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=319.41,781.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=471.07,809.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=525.46,861.71");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=542.69,769.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=595.31,819.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=667.77,872.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=610.61,946.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=636.75,766.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=590.22,713.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=683.42,717.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=705.92,788.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=785.75,873.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=808.26,763.25");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=900.20,738.99");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=874.52,803.05");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=741.35,981.27");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=803.74,939.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=871.88,1006.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=851.02,895.44");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=941.10,876.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=913.23,962.06");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=982.01,1010.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1031.15,1058.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1063.97,989.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1102.77,900.77");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1040.98,923.46");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1126.10,842.04");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1053.71,803.57");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=974.08,818.51");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1191.23,940.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=982.06,741.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1014.79,648.77");
    }

}

