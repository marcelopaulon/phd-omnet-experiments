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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=118.96,178.95");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=177.63,126.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=232.29,200.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=321.08,211.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=402.48,255.96");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=418.74,348.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=357.76,377.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=418.94,425.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=522.84,380.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=510.96,467.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=442.85,523.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=293.80,434.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=379.86,481.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=557.49,530.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=640.97,398.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=710.89,483.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=768.52,429.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=864.23,404.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=873.06,343.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=782.68,336.02");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=898.38,463.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=908.00,530.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=827.69,495.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=768.54,535.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=744.61,628.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=780.34,712.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=770.34,775.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=773.42,854.47");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=682.68,876.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=803.75,923.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=721.60,924.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=758.56,1014.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=699.88,1062.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=775.15,1076.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=771.69,1161.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=784.30,1234.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=716.85,1249.46");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=813.55,1303.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=822.52,1390.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=914.98,1390.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=995.26,1391.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1032.75,1329.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1004.27,1244.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1062.38,1270.67");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1136.19,1319.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1180.71,1251.23");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1119.20,1186.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=953.80,1338.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=923.23,1210.17");
    }

}

