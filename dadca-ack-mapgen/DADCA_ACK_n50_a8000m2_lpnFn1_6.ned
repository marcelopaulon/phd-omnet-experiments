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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=152.00,133.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=206.68,75.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=278.06,132.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=133.24,23.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=124.54,245.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=53.72,228.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=57.57,345.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=146.13,347.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=15.61,62.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=12.18,123.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=345.74,51.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=370.66,114.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=451.83,90.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=535.52,194.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=551.83,127.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=626.42,70.21");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=657.74,123.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=738.87,112.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=783.72,62.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=839.13,133.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=758.82,191.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=797.42,254.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=779.04,340.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=840.44,338.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=902.22,413.65");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=976.00,386.41");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=955.55,457.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=900.39,508.79");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=825.29,517.76");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=795.99,451.94");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=916.69,351.52");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=972.65,566.95");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=996.83,625.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=995.76,693.08");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1058.84,683.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1029.77,777.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1053.40,848.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=879.44,596.68");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1029.44,542.68");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1065.38,596.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1128.64,652.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1141.62,569.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1202.44,563.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1171.59,469.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1201.19,381.43");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1144.43,412.95");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1112.49,346.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1058.40,303.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=968.46,285.66");
    }

}

