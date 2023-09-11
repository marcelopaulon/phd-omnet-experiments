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
        @display("i=misc/sensor,p=86.53,162.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=24.01,117.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=59.73,48.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=200.68,151.02");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=231.84,212.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=194.68,287.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=121.75,308.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=72.29,255.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=215.75,353.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=125.93,410.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=71.94,440.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=128.81,476.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=61.95,361.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=1.44,447.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=34.00,504.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=18.32,315.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=298.46,311.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=359.33,320.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=359.06,416.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=408.36,281.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=443.15,355.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=467.41,424.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=539.73,357.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=504.95,290.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=593.84,281.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=666.44,336.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=671.68,270.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=755.86,384.04");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=671.58,421.67");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=290.47,436.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=286.77,496.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=248.85,561.18");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=153.44,532.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=141.77,593.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=221.95,648.04");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=152.84,691.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=183.53,767.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=99.66,822.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=177.72,829.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=155.71,889.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=215.08,880.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=60.40,939.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=57.60,999.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=146.71,1078.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=212.49,1048.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=267.64,1079.62");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=234.88,1141.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=229.99,1253.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=133.97,1272.88");
    }

}

