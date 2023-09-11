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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=31.78,127.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=23.34,186.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=103.07,215.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=57.08,280.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=117.75,286.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=159.81,334.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=158.07,422.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=87.12,352.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=8.81,369.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=3.60,448.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=65.27,451.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=219.01,431.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=272.18,229.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=229.35,273.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=179.49,140.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=233.77,112.16");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=310.33,81.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=241.06,50.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=306.73,289.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=343.37,359.33");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=346.56,203.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=399.99,78.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=218.95,189.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=15.40,15.28");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=94.02,16.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=434.71,201.58");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=423.81,287.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=477.88,323.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=545.14,386.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=523.92,460.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=459.40,464.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=409.79,378.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=320.00,471.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=280.43,409.70");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=160.09,492.27");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=80.11,545.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=24.85,578.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=88.22,626.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=114.62,719.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=104.83,801.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=189.42,753.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=214.32,697.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=187.51,639.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=36.28,669.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=34.89,793.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=43.03,860.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=126.62,904.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=148.98,960.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=159.70,1027.16");
    }

}

