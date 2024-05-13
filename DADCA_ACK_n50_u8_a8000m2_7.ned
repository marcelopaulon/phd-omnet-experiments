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

network DADCA_ACK_n50_u8_a8000m2_7
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
        @display("i=misc/sensor;p=775.16,7835.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1161.16,7930.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1765.25,7887.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2405.27,7929.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3601.92,7852.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3978.32,7981.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=435.23,7692.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1047.85,7574.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2037.31,7592.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4360.39,7534.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4722.53,7696.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5957.15,7715.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=264.94,7288.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=618.44,7289.38");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1519.50,7419.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3184.24,7307.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3858.21,7492.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5183.30,7395.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5573.47,7321.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1815.23,7226.64");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2164.88,7187.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2669.35,7251.18");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3708.70,7163.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4121.83,7195.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4760.98,7054.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=231.26,6767.45");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1131.18,6878.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1577.94,6804.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2399.85,6910.43");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2803.15,6843.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3184.94,6800.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3549.21,6809.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3923.65,6805.89");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5112.04,6780.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5504.21,6898.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=933.85,6553.20");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1887.54,6612.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4443.33,6732.83");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5820.60,6716.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=547.94,6488.40");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2464.01,6506.19");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2844.75,6466.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3617.01,6308.61");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5587.20,6367.67");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=155.47,6236.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1387.62,6252.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3089.64,6203.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4292.88,6069.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4928.77,6254.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5451.14,6038.53");
    }

}

