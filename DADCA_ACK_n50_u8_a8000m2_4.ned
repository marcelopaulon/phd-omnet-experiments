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

network DADCA_ACK_n50_u8_a8000m2_4
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
        @display("i=misc/sensor;p=590.51,7974.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1037.82,7792.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1402.97,7890.68");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1870.98,7872.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2740.96,7969.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3626.07,7782.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5107.49,7850.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5695.41,7848.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=53.32,7524.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=509.08,7578.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1315.08,7510.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1712.30,7520.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2237.74,7656.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2708.66,7579.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3240.69,7618.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4273.25,7713.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=905.28,7439.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2037.28,7367.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2442.96,7274.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3555.14,7400.50");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3947.30,7303.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4639.38,7388.05");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5204.11,7450.73");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5991.32,7452.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=632.56,7175.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1341.91,7122.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3052.63,7112.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5665.39,7066.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=79.16,6841.78");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1025.73,6837.54");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1661.50,6942.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2201.46,6763.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2625.67,6826.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3669.64,6924.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4000.90,6790.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4480.51,6903.40");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4887.70,6788.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5240.31,6767.71");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=572.64,6752.86");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3213.82,6660.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5667.23,6648.84");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=150.59,6398.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=514.40,6296.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1334.43,6361.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1740.09,6431.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2434.74,6313.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2831.10,6459.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3486.78,6420.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3904.10,6269.26");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4362.70,6411.23");
    }

}

