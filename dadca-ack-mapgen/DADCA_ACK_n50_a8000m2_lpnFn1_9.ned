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
        @display("i=misc/sensor,p=34.22,145.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=130.86,157.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=185.58,95.61");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=268.29,151.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=311.88,90.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=386.25,147.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=443.56,174.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=496.71,91.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=524.84,0.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=616.63,104.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=680.80,180.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=738.85,197.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=782.70,154.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=805.44,65.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=734.69,8.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=896.59,38.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=918.10,95.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=846.48,124.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=743.22,93.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=993.18,137.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=947.09,4.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=1023.04,33.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=1081.37,145.65");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=1160.71,131.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=1182.41,217.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=1199.28,284.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=1259.41,230.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=1274.87,310.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=1272.97,387.48");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=1294.54,469.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=1378.24,450.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1401.45,507.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1291.58,609.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1291.31,688.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1365.95,683.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1437.43,685.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1390.60,755.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1284.98,783.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1370.61,859.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1456.83,824.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1522.15,830.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1492.42,924.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1413.89,917.58");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1335.42,936.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1260.95,963.44");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1173.45,973.56");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1304.17,1036.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1201.45,900.32");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1225.29,1055.46");
    }

}

