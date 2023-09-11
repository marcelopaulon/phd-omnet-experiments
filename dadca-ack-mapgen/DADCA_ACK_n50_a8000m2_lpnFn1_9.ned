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
        @display("i=misc/sensor,p=111.67,40.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=180.66,30.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=251.12,65.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=315.38,20.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=420.75,63.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=478.85,105.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=536.04,164.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=563.58,100.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=602.99,31.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=684.54,35.03");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=765.93,47.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=738.41,101.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=752.75,191.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=677.14,144.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=669.37,218.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=891.26,204.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=841.71,164.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=941.57,76.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=1031.66,118.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=1078.93,74.67");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=1156.88,91.48");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=1235.65,56.71");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=1193.27,201.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=1261.90,153.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=1298.13,213.73");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=1211.06,120.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=1321.86,96.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=1367.99,40.22");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=1430.85,115.42");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=1516.90,77.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=1487.10,141.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=1447.56,202.48");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=1381.92,224.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=1258.19,293.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1315.98,361.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1281.66,449.21");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1336.92,523.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1278.97,555.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1328.34,637.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1285.37,713.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1351.64,733.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1336.70,798.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1430.62,812.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1425.59,881.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1417.78,752.40");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1322.20,982.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1399.68,967.49");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1481.00,964.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1508.51,1029.84");
    }

}

