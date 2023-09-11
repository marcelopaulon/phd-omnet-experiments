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
        @display("i=misc/sensor,p=193.40,119.82");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=112.69,10.05");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=205.93,49.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=304.11,30.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=110.57,168.34");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=20.59,131.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=182.67,190.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=142.75,239.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=188.63,295.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=292.76,207.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=245.94,261.41");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=274.37,133.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=399.94,135.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=426.77,219.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=461.12,303.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=429.80,393.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=348.12,420.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=353.32,342.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=286.22,352.19");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=214.40,390.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=209.66,460.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=192.24,539.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=130.20,538.73");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=292.71,457.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=278.21,530.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=280.84,617.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=327.68,674.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=398.62,738.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=407.88,799.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=480.52,805.75");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=560.90,835.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=548.49,774.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=564.94,704.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=618.57,853.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=486.45,891.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=442.11,968.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=373.96,1025.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=281.45,1047.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=266.04,985.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=183.22,1045.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=332.62,969.74");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=494.43,1002.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=545.54,957.73");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=447.50,1117.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=325.71,1130.61");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=205.83,976.29");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=290.39,921.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=233.16,816.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=333.85,870.64");
    }

}

