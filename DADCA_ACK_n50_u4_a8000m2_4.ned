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

network DADCA_ACK_n50_u4_a8000m2_4
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
        @display("i=misc/sensor;p=1458.49,7983.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2492.31,7967.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2949.90,7809.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3564.54,7992.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4723.54,7982.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5732.65,7992.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1033.64,7587.86");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1954.75,7628.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3324.10,7574.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3750.75,7634.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4844.78,7544.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5451.19,7557.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5824.33,7633.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1507.13,7360.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1855.06,7264.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2589.64,7355.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2998.20,7446.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3914.19,7281.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4445.39,7498.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=537.17,7194.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1052.02,7055.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2327.40,7048.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4744.23,7163.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5314.83,7010.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5771.22,7236.34");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=581.50,6769.90");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1423.36,6860.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1894.43,6819.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2731.18,6985.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3265.47,6980.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4311.51,6876.46");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4836.69,6774.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=207.23,6533.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2219.97,6668.32");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3812.56,6609.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5243.67,6518.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5723.30,6686.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=521.86,6292.76");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=968.28,6330.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1665.84,6391.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2659.06,6451.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3285.50,6398.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4308.22,6455.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4789.34,6312.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2217.60,6228.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5571.40,6038.40");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5949.92,6073.94");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=57.20,5919.99");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1078.41,5950.86");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3300.50,5993.01");
    }

}

