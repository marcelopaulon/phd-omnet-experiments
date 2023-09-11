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

network DADCA_ACK_n50_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=121.01,194.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=84.08,250.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=183.03,186.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=12.41,297.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=18.35,359.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=108.41,377.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=34.10,463.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=98.97,537.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=131.53,463.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=157.52,332.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=180.00,260.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=208.01,467.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=224.73,400.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=306.44,349.76");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=238.01,332.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=364.88,323.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=274.76,281.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=379.76,210.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=431.28,157.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=510.16,134.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=562.90,80.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=526.79,14.15");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=589.88,174.53");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=576.90,238.15");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=663.90,271.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=688.19,199.19");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=683.33,112.04");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=763.16,144.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=740.79,63.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=560.51,309.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=497.40,303.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=466.98,355.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=376.58,407.66");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=599.60,397.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=635.09,481.93");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=693.45,419.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=478.06,222.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=445.45,42.96");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=327.89,59.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=394.69,92.04");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=232.82,7.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=268.41,93.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=834.61,118.07");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=891.73,89.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=847.62,21.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=820.26,218.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=897.56,215.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=945.01,162.23");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1013.44,181.43");
    }

}

