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

network DADCA_ACK_n50_u16_a8000m2_6
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
        @display("i=misc/sensor;p=577.59,7855.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1032.79,7815.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1521.18,7991.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1910.59,7903.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2892.77,7888.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3583.35,7858.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4842.03,7881.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5377.29,7937.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5776.95,7944.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=58.97,7709.78");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3305.25,7534.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4260.60,7703.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5924.62,7608.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=612.17,7301.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1193.54,7304.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1537.30,7390.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2409.48,7428.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2770.77,7381.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3872.14,7415.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5457.03,7394.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1873.42,7241.86");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2522.02,7049.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2953.74,7019.60");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4060.67,7070.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4531.48,7068.76");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4890.31,7184.76");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=178.97,6815.59");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1179.04,6806.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2046.93,6918.65");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3478.47,6972.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5259.54,6777.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5612.79,6918.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=831.33,6683.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1709.02,6722.24");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2647.61,6704.24");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3798.87,6719.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4162.82,6635.04");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4718.92,6519.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=574.84,6358.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1469.13,6303.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2143.70,6286.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2955.57,6479.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3498.91,6412.38");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5723.81,6490.57");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=82.37,6165.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1071.72,6111.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1749.62,6077.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4436.10,6171.29");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5151.32,6230.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5611.42,6087.75");
    }

}

