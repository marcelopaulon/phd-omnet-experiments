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

network DADCA_ACK_n50_a8000m2_4
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
        @display("i=misc/sensor;p=69.52,7975.08");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=179.95,7989.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=294.84,7972.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=385.22,7965.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=605.67,7985.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=665.00,7960.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=753.62,7997.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=156.93,7928.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=446.26,7933.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=550.01,7919.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=781.72,7914.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=842.17,7954.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=907.19,7935.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=13.31,7876.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=99.57,7882.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=213.16,7903.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=309.38,7896.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=601.26,7883.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=731.45,7878.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=162.48,7860.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=230.56,7838.06");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=441.63,7853.14");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=775.78,7833.36");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=844.98,7863.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=909.28,7867.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=64.32,7785.26");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=148.54,7792.30");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=299.29,7785.11");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=379.02,7793.65");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=495.31,7784.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=554.14,7810.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=628.13,7822.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=432.46,7755.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=709.59,7778.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=772.87,7753.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=862.31,7760.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=7.64,7724.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=130.58,7727.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=242.22,7735.71");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=345.06,7729.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=576.63,7716.72");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=659.17,7732.21");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=838.23,7701.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=923.57,7702.94");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=83.78,7667.82");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=195.32,7661.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=270.72,7661.72");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=484.07,7693.51");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=701.02,7676.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=762.52,7684.75");
    }

}

