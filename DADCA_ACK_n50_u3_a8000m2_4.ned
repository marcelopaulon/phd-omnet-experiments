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

network DADCA_ACK_n50_u3_a8000m2_4
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
        @display("i=misc/sensor;p=591.71,7815.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1745.17,7996.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2183.64,7848.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2864.27,7931.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4192.63,7971.73");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4933.97,7999.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5687.44,7934.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1224.79,7648.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2536.87,7600.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2958.08,7547.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3772.63,7687.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4385.58,7591.91");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4946.74,7588.79");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=568.63,7447.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1611.62,7349.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2388.02,7266.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3356.13,7463.16");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3812.35,7308.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5550.62,7316.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1210.88,7233.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1997.55,7219.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2988.34,7177.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4167.64,7074.68");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4580.20,7177.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=262.62,6909.37");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=797.22,6938.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1722.27,6801.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2524.69,6889.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3180.89,6845.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3600.09,6887.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4446.33,6851.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4855.02,6910.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5525.47,6876.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1210.51,6620.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2189.23,6685.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3575.85,6527.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3937.63,6727.04");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=34.41,6439.40");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=473.75,6372.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1534.46,6427.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2311.16,6321.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2708.93,6277.07");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3010.82,6456.17");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4044.17,6373.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4595.84,6502.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5085.47,6495.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5793.10,6328.51");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=948.81,6054.82");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1335.23,6070.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1854.06,6076.13");
    }

}

